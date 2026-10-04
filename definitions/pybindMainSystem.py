#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN definition file
#
# Details:  the MainSystem class.
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
from definitionTypes import Deprecated #every deprecated function: since and the year of removal (#2807)

pb = PybindInterface()

# pb.DefFinishTable()

# #now finalize pybind class, but do nothing on latex side (sL1 ignored)






#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
pb.CreateNewRSTfile('MainSystem')
classStr = 'MainSystem'
pb.DefPyStartClass(classStr, classStr, 
                    r"""MainSystem is the class which defines a (multibody) system and it's instance if usually called `mbs`. Interactions with the system are done via MainSystem, either through, e.g., `mbs.AddObject(...)` or with create functions, such as `mbs.CreateRigidBody(...)`; States are accessible via `mbs.systemData`. The MainSystem shall only be created from a SystemContainer `SC` using `SC.AddSystem()`; do not use `exu.MainSystem()`, as the latter one would not be linked to a SystemContainer. Having already a valid `mbs`, you may use `SC.AppendSystem(mbs)`. """,
                    forbidPythonConstructor=False)

pb.AddDocu(
            "In C++, there is a MainSystem (the part which links to Python) and a System (computational part). "+
            "For that reason, the name is MainSystem on the Python side, but it is often just called 'system'. "+
            "For compatibility, it is recommended to denote the variable holding this system as mbs, the multibody dynamics system. "+
            "It can be created, visualized and computed. Use the following functions for system manipulation.")

pb.AddDocuNotebook('python/Notebooks/reference/mainSystem.ipynb', part='mainSystem')

pb.DefStartTable(classStr)

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#GENERAL FUNCTIONS

pb.DefPyFunctionAccess(cClass=classStr, pyName='Assemble', cName='Assemble', 
                        description="assemble items (nodes, bodies, markers, loads, ...) of multibody system; Calls CheckSystemIntegrity(...), AssembleCoordinates(), AssembleLTGLists(), AssembleInitializeSystemCoordinates(), and AssembleSystemInitialize()",
                        returnType='None',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='AssembleCoordinates', cName='AssembleCoordinates', 
                        description="assemble coordinates: assign computational coordinates to nodes and constraints (algebraic variables)",
                        returnType='None',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='AssembleLTGLists', cName='AssembleLTGLists', 
                        description="build ABRV:LTG coordinate lists for objects (used to build global ODE2RHS, MassMatrix, etc. vectors and matrices) and store special object lists (body, connector, constraint, ...)",
                        returnType='None',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='AssembleInitializeSystemCoordinates', cName='AssembleInitializeSystemCoordinates', 
                        description="initialize all system-wide coordinates based on initial values given in nodes",
                        returnType='None',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='AssembleSystemInitialize', cName='AssembleSystemInitialize', 
                        description="initialize some system data, e.g., generalContact objects (searchTree, etc.)",
                        returnType='None',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='Reset', cName='Reset', 
                        description="reset all lists of items (nodes, bodies, markers, loads, ...) and temporary vectors; deallocate memory",
                        returnType='None',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetSystemContainer', cName='GetMainSystemContainer', 
                        description="return the systemContainer where the mainSystem (mbs) was created",
                        returnType='"SystemContainer"', #SystemContainer not known at this point for .pyi
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='WaitForUserToContinue', cName='WaitForUserToContinue', 
                        deprecated=Deprecated('1.11.0', 2029),
                        description="interrupt further computation until user input --> 'pause' function; this command runs a loop in the background to have active response of the render window, e.g., to open the visualization dialog or use the right-mouse-button; behaves similar as SC.WaitForRenderEngineStopFlag()",
                        argList=['printMessage','deprecationWarning'],
                        defaultArgs=['True','True'],
                        returnType='None',
                        addDocu=False,
                        )

#this function is not absolutely needed, but kept for some possible use case with several mbs
pb.DefPyFunctionAccess(cClass=classStr, pyName='SendRedrawSignal', cName='SendRedrawSignal', 
                        description="this function is used to send a signal to the renderer that the scene shall be redrawn because the visualization state has been updated",
                        returnType='None',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetRenderEngineStopFlag', cName='GetRenderEngineStopFlag', 
                        description="get the current stop simulation flag; True=user wants to stop simulation",
                        returnType='bool',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='SetRenderEngineStopFlag', cName='SetRenderEngineStopFlag', 
                        description="set the current stop simulation flag; set to False, in order to continue a previously user-interrupted simulation",
                        argList=['stopFlag'],
                        argTypes=['bool'],
                        returnType='None',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='ActivateRendering', cName='ActivateRendering', 
                        description="activate (flag=True) or deactivate (flag=False) rendering for this system",
                        argList=['flag'],
                        argTypes=['bool'],
                        defaultArgs=['True'],
                        returnType='None',
                        )

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#USER FUNCTIONS
pb.DefPyFunctionAccess(cClass=classStr, pyName='SetPreStepUserFunction', cName='PySetPreStepUserFunction', 
                        description="Sets a user function PreStepUserFunction(mbs, t) executed at beginning of every computation step; in normal case return True; return False to stop simulation after current step; set to 0 (integer) in order to erase user function. Note that the time t in the args is already the end of the step, which allows to compute forces consistently with trapezoidal integrators; for higher order Runge-Kutta methods, step time will be available only in object-user functions. The PreStepUserFunction is recommended e.g., for prescribing forces or set values of actuators",
                        example = r"""def PreStepUserFunction(mbs, t):\\ \TAB print(mbs.systemData.NumberOfNodes())\\ \TAB if(t>1): \\ \TAB  \TAB return False \\ \TAB return True \\mbs.SetPreStepUserFunction(PreStepUserFunction)""",
                        argList=['value'],
                        argTypes=['Callable[["MainSystem", float],bool]'], #MainSystem not known at this point for .pyi
                        returnType='None',
                        )
                                                      
pb.DefPyFunctionAccess(cClass=classStr, pyName='GetPreStepUserFunction', cName='PyGetPreStepUserFunction', 
                        description="Returns the preStepUserFunction.",
                        argList=['asDict'],
                        argTypes=['bool'],
                        defaultArgs=['False'],
                        returnType='Callable[["MainSystem", float],bool]', #MainSystem not known at this point for .pyi
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='SetPostStepUserFunction', cName='PySetPostStepUserFunction', 
                        description="Sets a user function PostStepUserFunction(mbs, t) executed at end of every computation step; in normal case return True; return False to stop simulation after current step; set to 0 (integer) in order to erase user function. The difference to PreStepUserFunction, the PostStepUserFunction is called after the step has been computed, AFTER the discontinuous iterations, just BEFORE writing solution file, sensors and visualization. This allows to change or evaluate results before they are stored (e.g., do some projection).",
                        example = r"""def PostStepUserFunction(mbs, t):\\ \TAB print(mbs.systemData.NumberOfNodes())\\ \TAB if(t>1): \\ \TAB  \TAB return False \\ \TAB return True \\mbs.SetPostStepUserFunction(PostStepUserFunction)""",
                        argList=['value'],
                        argTypes=['Callable[["MainSystem", float],bool]'], #MainSystem not known at this point for .pyi
                        returnType='None',
                        )
                                                      
pb.DefPyFunctionAccess(cClass=classStr, pyName='GetPostStepUserFunction', cName='PyGetPostStepUserFunction', 
                        description="Returns the postStepUserFunction.",
                        argList=['asDict'],
                        argTypes=['bool'],
                        defaultArgs=['False'],
                        returnType='Callable[["MainSystem", float],bool]', #MainSystem not known at this point for .pyi
                        )
                                                      
pb.DefPyFunctionAccess(cClass=classStr, pyName='SetPostNewtonUserFunction', cName='PySetPostNewtonUserFunction', 
                        description="Sets a user function PostNewtonUserFunction(mbs, t) executed after successful Newton iteration in implicit or static solvers and after step update of explicit solvers, but BEFORE PostNewton functions are called by the solver; function returns list [discontinuousError, recommendedStepSize], containing a error of the PostNewtonStep, which is compared to [solver].discontinuous.iterationTolerance. The recommendedStepSize shall be negative, if no recommendation is given, 0 in order to enforce minimum step size or a specific value to which the current step size will be reduced and the step will be repeated; use this function, e.g., to reduce step size after impact or change of data variables; set to 0 (integer) in order to erase user function. Similar described by Flores and Ambrosio, https://doi.org/10.1007/s11044-010-9209-8",
                        example = r"""def PostNewtonUserFunction(mbs, t):\\ \TAB if(t>1): \\ \TAB  \TAB return [0, 1e-6] \\ \TAB return [0,0] \\mbs.SetPostNewtonUserFunction(PostNewtonUserFunction)""",
                        argList=['value'],
                        argTypes=['Callable[["MainSystem", float],[float,float]]'], #MainSystem not known at this point for .pyi
                        returnType='None',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetPostNewtonUserFunction', cName='PyGetPostNewtonUserFunction', 
                        description="Returns the postNewtonUserFunction.",
                        argList=['asDict'],
                        argTypes=['bool'],
                        defaultArgs=['False'],
                        returnType='Callable[["MainSystem", float],bool]', #MainSystem not known at this point for .pyi
                        )


pb.DefPyFunctionAccess(cClass=classStr, pyName='SetPreNewtonResidualUserFunction', cName='PySetPreNewtonResidualUserFunction', 
                        description="Sets a user function PreNewtonResidualUserFunction(mbs, t, newtonIt, discontinuousIt) executed prior to computation of the Newton residual in implicit or static solvers. This function returns nothing. The arguments newtonIt and discontinuousIt may be used to distinguish if the call is done at the beginning of a discontinuous iteration (newtonIt=0) or during Newton iterations (newtonIt>0). The typical use case would be to modify objects or loads in every iteration. Note that this user function is not called during Jacobian computation. If needed, the jacobian can be modified with the user function set by SetSystemJacobianUserFunction.",
                        example = r"""def PreNewtonResidualUserFunction(mbs, t, newtonIt, discontinuousIt):\\ \TAB print("t=",t,", newtonIt=",newtonIt,", discIt=",discontinuousIt)\\mbs.SetPreNewtonResidualUserFunction(PreNewtonResidualUserFunction)""",
                        argList=['value'],
                        argTypes=['Callable[["MainSystem", float, int, int],None]'], #MainSystem not known at this point for .pyi
                        returnType='None',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetPreNewtonResidualUserFunction', cName='PyGetPreNewtonResidualUserFunction', 
                        description="Returns the preNewtonResidualUserFunction.",
                        argList=['asDict'],
                        argTypes=['bool'],
                        defaultArgs=['False'],
                        returnType='Callable[["MainSystem", float, int, int],None]', #MainSystem not known at this point for .pyi
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='SetSystemJacobianUserFunction', cName='PySetSystemJacobianUserFunction', 
                        description="Sets a user function SystemJacobianUserFunction(mbs, t, factorODE2, factorODE2_t, factorODE1) executed after computation of the Newton jacobian of a static solver or an implicit timeintegrator; The function shall return additional terms for the jacobian at RHS, e.g., related to dependencies that are added by the user in the PreNewtonResidualUserFunction; RHS means that for a spring with stiffness K, the jacobian would be -K as it is computed for the RHS, see the RHS-LHS convention. If you like to completely replace the jacobian, consider using the solver's user function SetUserFunctionComputeNewtonJacobian which can be used to replace the jacobian computation; the factors factorODE2, factorODE2_t, factorODE1 must be multiplied with quantities related to ODE2 coordinates (like stiffness terms), ODE2_t velocity coordinates (like damping terms) and ODE1 quantities. The functions returns a MatrixContainer, for which the sparse format is recommended for efficiency reasons.",
                        example = r"""def SystemJacobianUserFunction(mbs, t, factorODE2, factorODE2_t, factorODE1):\\ \TAB return MatrixContainer([[factorODE2*10,0],[0,0]])\\mbs.SetSystemJacobianUserFunction(SystemJacobianUserFunction)""",
                        argList=['value'],
                        argTypes=['Callable[["MainSystem", float, float, float, float],'+matrixContainerType+']'], #MainSystem not known at this point for .pyi
                        returnType='None',
                        )
                                                      
pb.DefPyFunctionAccess(cClass=classStr, pyName='GetSystemJacobianUserFunction', cName='PyGetSystemJacobianUserFunction', 
                        description="Returns the systemJacobianUserFunction.",
                        argList=['asDict'],
                        argTypes=['bool'],
                        defaultArgs=['False'],
                        returnType='Callable[["MainSystem", float, float, float, float],'+matrixContainerType+']', #MainSystem not known at this point for .pyi
                        )

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                                                      

#contact:                                      
pb.DefPyFunctionAccess(cClass=classStr, pyName='AddGeneralContact', cName='AddGeneralContact', 
                        description="add a new general contact, used to enable efficient contact computation between objects (nodes or markers)", 
                        options='py::return_value_policy::reference_internal',
                        returnType='GeneralContact',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetGeneralContact', cName='GetGeneralContact', 
                        description="get read/write access to GeneralContact with index generalContactNumber stored in mbs; Examples shows how to access the GeneralContact object added with last AddGeneralContact() command:",
                        example = 'gc=mbs.GetGeneralContact(mbs.NumberOfGeneralContacts()-1)',
                        argList=['generalContactNumber'],
                        options='py::return_value_policy::reference_internal',
                        argTypes=['int'],
                        returnType='GeneralContact',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='DeleteGeneralContact', cName='DeleteGeneralContact', 
                        description="delete GeneralContact with index generalContactNumber in mbs; other general contacts are resorted (index changes!)",
                        argList=['generalContactNumber'],
                        argTypes=['int'],
                        returnType='None',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='NumberOfGeneralContacts', cName='NumberOfGeneralContacts', 
                        description="Return number of GeneralContact objects in mbs", 
                        returnType='int',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetAvailableFactoryItems', cName='GetAvailableFactoryItems', 
                                description="get all available items to be added (nodes, objects, etc.); this is useful in particular in case of additional user elements to check if they are available; the available items are returned as dictionary, containing lists of strings for Node, Object, etc.",
                                returnType='dict',
                                )


#++++++++++++++++++++++++++++++++++++++++++++++++++
#see: https://pybind11.readthedocs.io/en/stable/upgrade.html
pb.DefPyFunctionAccess(cClass=classStr, pyName='GetDictionary', cName='GetDictionary', 
                        description="[UNDER DEVELOPMENT]: return the dictionary of the system data (todo: and state), e.g., to copy the system or for pickling",
                        argList=[],
                        argTypes=[],
                        returnType='dict',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='SetDictionary', cName='SetDictionary', 
                        description="[UNDER DEVELOPMENT]: set system data (todo: and state) from given dictionary; used for pickling",
                        argList=['systemDict'],
                        argTypes=['dict'],
                        returnType='None',
                        )

pb.CppCode(pickleDictTemplateNew.replace('{ClassName}', classStr))
#in C++:
        # .def(py::pickle(
        #     [](const MainSystem& self) {
        #         return py::make_tuple(self.GetDictionary());
        #     },
        #     [](const py::tuple& t) {
        #         CHECKandTHROW(t.size() == 1, "MainSystem: loading data with pickle received invalid data structure!");

        #         MainSystem* self = new MainSystem();
        #         //self.SetDictionary(t[0].cast<py::dict>());
        #         self->SetDictionary(py::cast<py::dict>(t[0]));

        #         return self;
        #     }))


#++++++++++++++++++++++++++++++++++++++++++++++++++

#old version, with variables: pb.DefPyFunctionAccess(cClass=classStr, pyName='__repr__', cName='[](const MainSystem &ms) {\n            return "<systemData: \\n" + ms.GetMainSystemData().PyInfoSummary() + "\\nmainSystem:\\n  variables = " + EXUstd::ToString(ms.variables) + "\\n  sys = " + EXUstd::ToString(ms.systemVariables) + "\\n>\\n"; }', 
pb.DefPyFunctionAccess(cClass=classStr, pyName='__repr__', cName=r"""[](const MainSystem &ms) {
            return "<systemData: \n" + ms.GetMainSystemData().PyInfoSummary() + "\nFor details see mbs.systemData, mbs.sys and mbs.variables\n>\n"; }""", 
                        description="return the representation of the system, which can be, e.g., printed",
                        isLambdaFunction = True,
                        example = 'print(mbs)')

pb.CppCode('        .def_property("systemIsConsistent", &MainSystem::GetFlagSystemIsConsistent, &MainSystem::SetFlagSystemIsConsistent)\n') 
pb.DefDataAccess('systemIsConsistent','this flag is used by solvers to decide, whether the system is in a solvable state; this flag is set to False as long as Assemble() has not been called; any modification to the system, such as Add...(), Modify...(), etc. will set the flag to False again; this flag can be modified (set to True), if a change of e.g.~an object (change of stiffness) or load (change of force) keeps the system consistent, but would normally lead to systemIsConsistent=False',
                       dataType='bool',
                       )

pb.CppCode('        .def_property("interactiveMode", &MainSystem::GetInteractiveMode, &MainSystem::SetInteractiveMode)\n') 
pb.DefDataAccess('interactiveMode','set this flag to True in order to invoke a Assemble() command in every system modification, e.g., AddNode, AddObject, ModifyNode, ...; this helps that the system can be visualized in interactive mode.',
                       dataType='bool',
                       )

pb.CppCode('        .def_readwrite("variables", &MainSystem::variables, py::return_value_policy::reference)\n') 
pb.DefDataAccess('variables','this dictionary may be used by the user to store model-specific data, in order to avoid global Python variables in complex models; mbs.variables["myvar"] = 42 ',
                       dataType='dict',
                       )

pb.CppCode('        .def_readwrite("sys", &MainSystem::systemVariables, py::return_value_policy::reference)\n') 
pb.DefDataAccess('sys','this dictionary is used by exudyn Python libraries, e.g., solvers, to avoid global Python variables ',
                       dataType='dict',
                       )

pb.CppCode('        .def_property("solverSignalJacobianUpdate", &MainSystem::GetFlagSolverSignalJacobianUpdate, &MainSystem::SetFlagSolverSignalJacobianUpdate)\n') 
pb.DefDataAccess('solverSignalJacobianUpdate','this flag is used by solvers to decide, whether the jacobian should be updated; at beginning of simulation and after jacobian computation, this flag is set automatically to False; use this flag to indicate system changes, e.g., during time integration  ',
                       dataType='bool',
                       )

pb.CppCode('        .def_readwrite("systemData", &MainSystem::mainSystemData, py::return_value_policy::reference)\n') 
pb.DefDataAccess('systemData','Access to SystemData structure; enables access to number of nodes, objects, ... and to (current, initial, reference, ...) state variables (ODE2, AE, Data,...)',
                       dataType='SystemData',
                       )

pb.DefFinishTable()#only finalize latex table

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#create extensions
pb.DefStartClass('MainSystem extensions (create)',r"""This section represents extensions to MainSystem, which are direct calls to Python functions; the 'create' extensions to simplify the creation of multibody systems, such as CreateMassPoint(...); these extensions allow a more intuitive interaction with the MainSystem class, see the following example. For activation, import `exudyn.misc.mainSystemExtensions` or `exudyn.utilities`""", subSection=True,labelName='sec:mainsystem:pythonExtensionsCreate')

pb.AddDocuNotebook('python/Notebooks/reference/mainSystem.ipynb', part='create')


pb.ExtensionMarkdown('MainSystemCreateExt') #written by tools/generators/mainSystemExtensionDocsEmitter.py

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#function extensions
pb.DefStartClass('MainSystem extensions (general)',r"""This section represents general extensions to MainSystem, which are direct calls to Python functions, such as PlotSensor or SolveDynamic; these extensions allow a more intuitive interaction with the MainSystem class, see the following example. For activation, import `exudyn.misc.mainSystemExtensions` or `exudyn.utilities`""", subSection=True,labelName='sec:mainsystem:pythonExtensions')

pb.AddDocuNotebook('python/Notebooks/reference/mainSystem.ipynb', part='extensions')


pb.ExtensionMarkdown('MainSystemExt') #written by tools/generators/mainSystemExtensionDocsEmitter.py


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#NODE
pb.CppCode("\n//        NODES:\n")
pb.DefStartClass(classStr+': Node','', subSection=True,labelName='sec:mainsystem:node')

pb.AddDocu('This section provides functions for adding, reading and modifying nodes. '+
            'Nodes are used to define coordinates (unknowns to the static system and degrees of freedom '+
            'if constraints are not present). Nodes can provide various types of coordinates for '+
            'second/first order differential equations (ODE2/ODE1), algebraic equations (AE) and for data '+
            '(history) variables -- which are not providing unknowns in the nonlinear solver but will be solved '+
            'in an additional nonlinear iteration for e.g., contact, friction or plasticity.')

pb.AddDocuNotebook('python/Notebooks/reference/mainSystem.ipynb', part='node')

pb.DefStartTable(classStr+':nodes')

pb.DefPyFunctionAccess(cClass=classStr, pyName='AddNode', cName='AddMainNodePyClass', 
                                description="add a node with nodeDefinition from Python node class; returns (global) node index (type NodeIndex) of newly added node; use int(nodeIndex) to convert to int, if needed (but not recommended in order not to mix up index types of nodes, objects, markers, ...)",
                                argList=['pyObject'],
                                example = r"""item = Rigid2D( referenceCoordinates= [1,0.5,0], initialVelocities= [10,0,0]) \\mbs.AddNode(item) \\nodeDict = {'nodeType': 'Point', \\'referenceCoordinates': [1.0, 0.0, 0.0], \\'initialCoordinates': [0.0, 2.0, 0.0], \\'name': 'example node'} \\mbs.AddNode(nodeDict)""",
                                argTypes=[itemDict],
                                returnType='NodeIndex',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='DeleteNode', cName='PyDeleteNode', 
                                description="delete the node with nodeNumber in MainSystem; consistently renames nodes according to their new node numbers; adapts node numbers in sensors and in markers; items using deleted nodeNumber obtain invalid nodeNumber",
                                argList=['nodeNumber','suppressWarnings'],
                                defaultArgs=['','False'],
                                argTypes=['NodeIndex','bool'],
                                example = "mbs.DeleteNode(nodeNumber=42)",
                                returnType='None',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetNodeNumber', cName='PyGetNodeNumber', 
                                description="get node's number by name (string)",
                                argList=['nodeName'],
                                example = "n = mbs.GetNodeNumber('example node')",
                                argTypes=['str'],
                                returnType='NodeIndex',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetNode', cName='PyGetNode',
                                description="get node's dictionary by node number (type NodeIndex)",
                                argList=['nodeNumber'],
                                example = "nodeDict = mbs.GetNode(0)",
                                argTypes=['NodeIndex'],
                                returnType='dict',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='ModifyNode', cName='PyModifyNode', 
                                description="modify node's dictionary by node number (type NodeIndex)",
                                argList=['nodeNumber','nodeDict'],
                                example = "mbs.ModifyNode(nodeNumber, nodeDict)",
                                argTypes=['NodeIndex', 'dict'],
                                returnType='None',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetNodeDefaults', cName='PyGetNodeDefaults', 
                                description="get node's default values for a certain nodeType as (dictionary)",
                                argList=['typeName'],
                                example = r"""nodeType = 'Point'\\nodeDict = mbs.GetNodeDefaults(nodeType)""",
                                argTypes=['str'],
                                returnType='dict',
                                )

#pb.DefPyFunctionAccess(cClass=classStr, pyName='CallNodeFunction', cName='PyCallNodeFunction', 
#                                description="call specific node function",
#                                argList=['nodeNumber', 'functionName', 'args'],
#                                defaultArgs=['', '', 'py::dict()']
#                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetNodeOutput', cName='PyGetNodeOutputVariable', 
                                description="get the ouput of the node specified with the OutputVariableType; output may be scalar or array (e.g., displacement vector)",
                                argList=['nodeNumber','variableType','configuration'],
                                defaultArgs=['','','exu.ConfigurationType::Current'],
                                example = "mbs.GetNodeOutput(nodeNumber=0, variableType=exu.OutputVariableType.Displacement)",
                                argTypes=['NodeIndex','OutputVariableType','ConfigurationType'],
                                returnType=returnedArray,
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetNodeODE2Index', cName='PyGetNodeODE2Index', 
                                description="get index in the global ODE2 coordinate vector for the first node coordinate of the specified node",
                                argList=['nodeNumber'],
                                example = "mbs.GetNodeODE2Index(nodeNumber=0)",
                                argTypes=['NodeIndex'],
                                returnType='int',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetNodeODE1Index', cName='PyGetNodeODE1Index', 
                                description="get index in the global ODE1 coordinate vector for the first node coordinate of the specified node",
                                argList=['nodeNumber'],
                                example = "mbs.GetNodeODE1Index(nodeNumber=0)",
                                argTypes=['NodeIndex'],
                                returnType='int',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetNodeAEIndex', cName='PyGetNodeAEIndex', 
                                description="get index in the global AE coordinate vector for the first node coordinate of the specified node",
                                argList=['nodeNumber'],
                                example = "mbs.GetNodeAEIndex(nodeNumber=0)",
                                argTypes=['NodeIndex'],
                                returnType='int',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetNodeParameter', cName='PyGetNodeParameter', 
                                description="get nodes's parameter from node number (type NodeIndex) and parameterName; parameter names can be found for the specific items in the reference manual; for visualization parameters, use a 'V' as a prefix",
                                argList=['nodeNumber', 'parameterName'],
                                example = "mbs.GetNodeParameter(0, 'referenceCoordinates')",
                                argTypes=['NodeIndex','str'],
                                returnType='Any',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='SetNodeParameter', cName='PySetNodeParameter', 
                                description="set parameter 'parameterName' of node with node number (type NodeIndex) to value; parameter names can be found for the specific items in the reference manual; for visualization parameters, use a 'V' as a prefix",
                                argList=['nodeNumber', 'parameterName', 'value'],
                                example = "mbs.SetNodeParameter(0, 'Vshow', True)",
                                argTypes=['NodeIndex','str','Any'],
                                returnType='None',
                                )

pb.DefFinishTable()
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#OBJECT
pb.CppCode("\n//        OBJECTS:\n")
pb.DefStartClass(classStr+': Object', '', subSection=True,labelName='sec:mainsystem:object')

pb.AddDocu('This section provides functions for adding, reading and modifying objects, which can be bodies (mass point, '+
            'rigid body, finite element, ...), connectors (spring-damper or joint) or general objects. Objects provided '+
            'terms to the residual of equations resulting from every coordinate given by the nodes. Single-noded objects '+
            '(e.g.~mass point) provides exactly residual terms for its nodal coordinates. Connectors constrain or penalize '+
            'two markers, which can be, e.g., position, rigid or coordinate markers. Thus, the dependence of objects is '+
            'either on the coordinates of the marker-objects/nodes or on nodes which the objects possess themselves.')

pb.AddDocuNotebook('python/Notebooks/reference/mainSystem.ipynb', part='object')

pb.DefStartTable(classStr+':objects')
#pb.DefPyFunctionAccess(cClass=classStr, pyName='AddObject', cName='[](MainSystem& mainSystem, py::object pyObject) {return mainSystem.AddMainObjectPyClass(pyObject); }', 
pb.DefPyFunctionAccess(cClass=classStr, pyName='AddObject', cName='AddMainObjectPyClass', 
                                description="add an object with objectDefinition from Python object class; returns (global) object number (type ObjectIndex) of newly added object",
                                argList=['pyObject'],
                                example = r"""item = MassPoint(name='heavy object', nodeNumber=0, mass=100) \\mbs.AddObject(item) \\objectDict = {'objectType': 'MassPoint', \\'mass': 10, \\'nodeNumber': 0, \\'name': 'example object'} \\mbs.AddObject(objectDict)""",
                                argTypes=[itemDict],
                                returnType='ObjectIndex',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='DeleteObject', cName='PyDeleteObject', 
                                description="delete the object with objectNumber in MainSystem; consistently renames objects according to their new object numbers; adapts object numbers in sensors and in markers; items using deleted objectNumber obtain invalid objectNumber; with the option deleteDependentItems (default=True) the function also delete nodes and markers which are used by the object",
                                argList=['objectNumber','deleteDependentItems','suppressWarnings'],
                                argTypes=['ObjectIndex','bool','bool'],
                                defaultArgs=['','True','False'],
                                example = "mbs.DeleteObject(objectNumber=42)",
                                returnType='None',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetObjectNumber', cName='PyGetObjectNumber', 
                                description="get object's number by name (string)",
                                argList=['objectName'],
                                example = "n = mbs.GetObjectNumber('heavy object')",
                                argTypes=['str'],
                                returnType='ObjectIndex',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetObject', cName='PyGetObject', 
                                description="get object's dictionary by object number (type ObjectIndex); NOTE: visualization parameters have a prefix 'V'; in order to also get graphicsData written, use addGraphicsData=True (which is by default False, as it would spoil the information)",
                                argList=['objectNumber','addGraphicsData'],
                                argTypes=['ObjectIndex','bool'],
                                defaultArgs=['','False'],
                                example = "objectDict = mbs.GetObject(0)",
                                returnType='dict',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='ModifyObject', cName='PyModifyObject', 
                                description="modify object's dictionary by object number (type ObjectIndex); NOTE: visualization parameters have a prefix 'V'",
                                argList=['objectNumber','objectDict'],
                                argTypes=['ObjectIndex','dict'],
                                example = "mbs.ModifyObject(objectNumber, objectDict)",
                                returnType='None',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetObjectDefaults', cName='PyGetObjectDefaults', 
                                description="get object's default values for a certain objectType as (dictionary)",
                                argList=['typeName'],
                                argTypes=['str'],
                                example = r"""objectType = 'MassPoint'\\objectDict = mbs.GetObjectDefaults(objectType)""",
                                returnType='dict',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetObjectOutput', cName='PyGetObjectOutputVariable', 
                                description="get object's current output variable from object number (type ObjectIndex) and OutputVariableType; for connectors, it can only be computed for exu.ConfigurationType.Current configuration!",
                                argList=['objectNumber', 'variableType', 'configuration'],
                                argTypes=['ObjectIndex','OutputVariableType','ConfigurationType'],
                                defaultArgs=['','','exu.ConfigurationType::Current'],
                                returnType=returnedArray,
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='Inspect', cName='PyInspect',
                                description="what an item provides and requests (#2203): for the typed index of an object, node, marker, load or sensor and a what of type exu.InspectType, a list of exported enumeration members - the output variables (OutputVariableType; energies only where they can be computed with the current parameters), the object, node or marker type flags (ObjectType, NodeType, MarkerType), the requested node types per node (for a node marker: requirements, each a list of alternatives) or marker types per marker, the access functions of a body (AccessFunctionType); with what=None a dict of all that apply to the item; a what that does not apply raises with the list of those that do",
                                argList=['itemIndex', 'what'],
                                argTypes=['Any', 'InspectType'],
                                defaultArgs=['', 'py::none()'],
                                example = r"""mbs.Inspect(oMassPoint, exu.InspectType.OutputVariables)\\mbs.Inspect(oSpringDamper) \#all that apply""",
                                returnType='Any',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='ComputeItem', cName='PyComputeItem',
                                description="compute a function of an item at the current state of the system (#2779) - for tests and the debugging of items: for the typed index of an object, node or marker and a what of type exu.ComputeItemType, a numpy array (a dict for the kinematics of a marker), in the coordinates of the item; localPosition is the position in a body, vector the force and torque of a Jacobian derivative; with what=None the list of what applies to the item; a what that does not apply raises with that list; the system must be assembled; exudyn.advancedUtilities.NumericalJacobian gives the numerical derivative to compare with",
                                argList=['itemIndex', 'what', 'localPosition', 'vector'],
                                argTypes=['Any', 'ComputeItemType', 'Vector3D', 'Any'],
                                defaultArgs=['', 'py::none()', '(std::vector<Real>)Vector3D({0.,0.,0.})', 'py::none()'],
                                example = r"""mbs.ComputeItem(oBody, exu.ComputeItemType.PositionJacobian, localPosition=[0.1,0,0])\\mbs.ComputeItem(oBody) \#what applies""",
                                returnType='Any',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetObjectOutputBody', cName='PyGetObjectOutputVariableBody', 
                                description="get body's output variable from object number (type ObjectIndex) and OutputVariableType, using the localPosition as defined in the body, and as used in MarkerBody and SensorBody",
                                argList=['objectNumber', 'variableType', 'localPosition', 'configuration'],
                                argTypes=['ObjectIndex','OutputVariableType',vector3D,'ConfigurationType'],
                                defaultArgs=['','','(std::vector<Real>)Vector3D({0,0,0})','exu.ConfigurationType::Current'],
                                example = "u = mbs.GetObjectOutputBody(objectNumber = 1, variableType = exu.OutputVariableType.Position, localPosition=[1,0,0], configuration = exu.ConfigurationType.Initial)",
                                returnType=returnedArray,
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetObjectOutputSuperElement', cName='PyGetObjectOutputVariableSuperElement', 
                                description="get output variable from mesh node number of object with type SuperElement (GenericODE2, FFRF, FFRFreduced - CMS) with specific OutputVariableType; the meshNodeNumber is the object's local node number, not the global node number!",
                                argList=['objectNumber', 'variableType', 'meshNodeNumber', 'configuration'],
                                argTypes=['ObjectIndex','OutputVariableType','int','ConfigurationType'],
                                defaultArgs=['','','','exu.ConfigurationType::Current'],
                                example = "u = mbs.GetObjectOutputSuperElement(objectNumber = 1, variableType = exu.OutputVariableType.Position, meshNodeNumber = 12, configuration = exu.ConfigurationType.Initial)",
                                returnType=returnedArray,
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetObjectParameter', cName='PyGetObjectParameter', 
                                description="get objects's parameter from object number (type ObjectIndex) and parameterName; parameter names can be found for the specific items in the reference manual; for visualization parameters, use a 'V' as a prefix; NOTE that BodyGraphicsData cannot be get or set, use dictionary access instead",
                                argList=['objectNumber', 'parameterName'],
                                argTypes=['ObjectIndex','str'],
                                example = "mbs.GetObjectParameter(objectNumber = 0, parameterName = 'nodeNumber')",
                                returnType='Any',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='SetObjectParameter', cName='PySetObjectParameter', 
                                description="set parameter 'parameterName' of object with object number (type ObjectIndex) to value;; parameter names can be found for the specific items in the reference manual; for visualization parameters, use a 'V' as a prefix; NOTE that BodyGraphicsData cannot be get or set, use dictionary access instead",
                                argList=['objectNumber', 'parameterName', 'value'],
                                argTypes=['ObjectIndex','str','Any'],
                                example = "mbs.SetObjectParameter(objectNumber = 0, parameterName = 'Vshow', value=True)",
                                returnType='None',
                                )

pb.DefFinishTable()

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#MARKER
pb.CppCode("\n//        MARKER:\n")
pb.DefStartClass(classStr+': Marker', '', subSection=True, labelName='sec:mainsystem:marker')

pb.AddDocu('This section provides functions for adding, reading and modifying markers. Markers define how to measure '+
            'primal kinematical quantities on objects or nodes (e.g., position, orientation or coordinates themselves), '+
            'and how to act on the quantities which are dual to the kinematical quantities (e.g., force, torque and '+
            'generalized forces). Markers provide unique interfaces for loads, sensors and constraints in order to address '+
            'these quantities independently of the structure of the object or node (e.g., rigid or flexible body).')

pb.AddDocuNotebook('python/Notebooks/reference/mainSystem.ipynb', part='marker')

pb.DefStartTable(classStr+':markers')

pb.DefPyFunctionAccess(cClass=classStr, pyName='AddMarker', cName='AddMainMarkerPyClass', 
                                description="add a marker with markerDefinition from Python marker class; returns (global) marker number (type MarkerIndex) of newly added marker",
                                argList=['pyObject'],
                                example = r"""item = MarkerNodePosition(name='my marker',nodeNumber=1) \\mbs.AddMarker(item)\\markerDict = {'markerType': 'NodePosition', \\  'nodeNumber': 0, \\  'name': 'position0'}\\mbs.AddMarker(markerDict)""",
                                argTypes=[itemDict],
                                returnType='MarkerIndex',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='DeleteMarker', cName='PyDeleteMarker', 
                                description="delete the marker with markerNumber in MainSystem; consistently renames markers according to their new marker numbers; adapts marker numbers in objects, loads and sensors; items using deleted markerNumber obtain invalid markerNumber",
                                argList=['markerNumber','suppressWarnings'],
                                defaultArgs=['','False'],
                                argTypes=['MarkerIndex','bool'],
                                example = "mbs.DeleteMarker(markerNumber=42)",
                                returnType='None',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetMarkerNumber', cName='PyGetMarkerNumber', 
                                description="get marker's number by name (string)",
                                argList=['markerName'],
                                example = "n = mbs.GetMarkerNumber('my marker')",
                                argTypes=['str'],
                                returnType='MarkerIndex',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetMarker', cName='PyGetMarker', 
                                description="get marker's dictionary by index",
                                argList=['markerNumber'],
                                example = "markerDict = mbs.GetMarker(0)",
                                argTypes=['MarkerIndex'],
                                returnType='dict',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='ModifyMarker', cName='PyModifyMarker', 
                                description="modify marker's dictionary by index",
                                argList=['markerNumber','markerDict'],
                                example = "mbs.ModifyMarker(markerNumber, markerDict)",
                                argTypes=['MarkerIndex','dict'],
                                returnType='None',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetMarkerDefaults', cName='PyGetMarkerDefaults', 
                                description="get marker's default values for a certain markerType as (dictionary)",
                                argList=['typeName'],
                                example = r"""markerType = 'NodePosition'\\markerDict = mbs.GetMarkerDefaults(markerType)""",
                                argTypes=['str'],
                                returnType='dict',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetMarkerParameter', cName='PyGetMarkerParameter', 
                                description="get markers's parameter from markerNumber and parameterName; parameter names can be found for the specific items in the reference manual",
                                argList=['markerNumber', 'parameterName'],
                                argTypes=['MarkerIndex','str'],
                                returnType='Any',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='SetMarkerParameter', cName='PySetMarkerParameter', 
                                description="set parameter 'parameterName' of marker with markerNumber to value; parameter names can be found for the specific items in the reference manual",
                                argList=['markerNumber', 'parameterName', 'value'],
                                argTypes=['MarkerIndex','str','Any'],
                                returnType='None',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetMarkerOutput', cName='PyGetMarkerOutputVariable', 
                                description="get the ouput of the marker specified with the OutputVariableType; currently only provides Displacement, Position and Velocity for position based markers, and RotationMatrix, Rotation and AngularVelocity(Local) for markers providing orientation; Coordinates and Coordinates_t available for coordinate markers",
                                argList=['markerNumber','variableType','configuration'],
                                defaultArgs=['','','exu.ConfigurationType::Current'],
                                example = "mbs.GetMarkerOutput(markerNumber=0, variableType=exu.OutputVariableType.Position)",
                                argTypes=['MarkerIndex','OutputVariableType','ConfigurationType'],
                                returnType=returnedArray,
                                )


pb.DefFinishTable()

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#LOAD
pb.CppCode("\n//        LOADS:\n")
pb.DefStartClass(classStr+': Load', '', subSection=True, labelName='sec:mainsystem:load')

pb.AddDocu('This section provides functions for adding, reading and modifying operating loads. '+
            'Loads are used to act on the quantities which are dual to the primal kinematic quantities, '+
            'such as displacement and rotation. Loads represent, e.g., forces, torques or generalized forces.')

pb.AddDocuNotebook('python/Notebooks/reference/mainSystem.ipynb', part='load')

pb.DefStartTable(classStr+':loads')

pb.DefPyFunctionAccess(cClass=classStr, pyName='AddLoad', cName='AddMainLoadPyClass', 
                                description="add a load with loadDefinition from Python load class; returns (global) load number (type LoadIndex) of newly added load",
                                argList=['pyObject'],
                                example = r"""item = mbs.AddLoad(LoadForceVector(loadVector=[1,0,0], markerNumber=0, name='heavy load')) \\mbs.AddLoad(item)\\loadDict = {'loadType': 'ForceVector',\\  'markerNumber': 0,\\  'loadVector': [1.0, 0.0, 0.0],\\  'name': 'heavy load'} \\mbs.AddLoad(loadDict)""",
                                argTypes=[itemDict],
                                returnType='LoadIndex',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='DeleteLoad', cName='PyDeleteLoad', 
                                description="delete the load with loadNumber in MainSystem; consistently renames loads according to their new load numbers; deleteDependentMarkers (default=True) also deletes the corresponding marker",
                                argList=['loadNumber', 'deleteDependentMarkers', 'suppressWarnings'],
                                defaultArgs=['','True','False'],
                                argTypes=['LoadIndex','bool','bool'],
                                example = "mbs.DeleteLoad(loadNumber=42)",
                                returnType='None',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetLoadNumber', cName='PyGetLoadNumber', 
                                description="get load's number by name (string)",
                                argList=['loadName'],
                                example = "n = mbs.GetLoadNumber('heavy load')",
                                argTypes=['str'],
                                returnType='LoadIndex',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetLoad', cName='PyGetLoad', 
                                description="get load's dictionary by index",
                                argList=['loadNumber'],
                                example = "loadDict = mbs.GetLoad(0)",
                                argTypes=['LoadIndex'],
                                returnType='dict',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='ModifyLoad', cName='PyModifyLoad', 
                                description="modify load's dictionary by index",
                                argList=['loadNumber','loadDict'],
                                example = "mbs.ModifyLoad(loadNumber, loadDict)",
                                argTypes=['LoadIndex','dict'],
                                returnType='None',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetLoadDefaults', cName='PyGetLoadDefaults', 
                                description="get load's default values for a certain loadType as (dictionary)",
                                argList=['typeName'],
                                example = r"""loadType = 'ForceVector'\\loadDict = mbs.GetLoadDefaults(loadType)""",
                                argTypes=['str'],
                                returnType='dict',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetLoadValues', cName='PyGetLoadValues', 
                                description="Get current load values, specifically if user-defined loads are used; can be scalar or vector-valued return value",
                                argList=['loadNumber'],
                                argTypes=['LoadIndex'],
                                returnType=returnedArray,
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetLoadParameter', cName='PyGetLoadParameter', 
                                description="get loads's parameter from loadNumber and parameterName; parameter names can be found for the specific items in the reference manual",
                                argList=['loadNumber', 'parameterName'],
                                argTypes=['LoadIndex','str'],
                                returnType='Any',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='SetLoadParameter', cName='PySetLoadParameter', 
                                description="set parameter 'parameterName' of load with loadNumber to value; parameter names can be found for the specific items in the reference manual",
                                argList=['loadNumber', 'parameterName', 'value'],
                                argTypes=['LoadIndex','str','Any'],
                                returnType='None',
                                )

pb.DefFinishTable()

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#SENSORS
pb.CppCode("\n//        SENSORS:\n")
pb.DefStartClass(classStr+': Sensor', '', subSection=True, labelName='sec:mainsystem:sensor')

pb.AddDocu('This section provides functions for adding, reading and modifying operating sensors. '+
            'Sensors are used to measure information in nodes, objects, markers, and loads for output in a file.')

pb.AddDocuNotebook('python/Notebooks/reference/mainSystem.ipynb', part='sensor')

pb.DefStartTable(classStr+':sensors')

pb.DefPyFunctionAccess(cClass=classStr, pyName='AddSensor', cName='AddMainSensorPyClass',
                                description="add a sensor with sensor definition from Python sensor class; returns (global) sensor number (type SensorIndex) of newly added sensor",
                                argList=['pyObject'],
                                example = r"""item = mbs.AddSensor(SensorNode(sensorType= exu.SensorType.Node, nodeNumber=0, name='test sensor')) \\mbs.AddSensor(item)\\sensorDict = {'sensorType': 'Node',\\  'nodeNumber': 0,\\  'fileName': 'sensor.txt',\\  'name': 'test sensor'} \\mbs.AddSensor(sensorDict)""",
                                argTypes=[itemDict],
                                returnType='SensorIndex',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='DeleteSensor', cName='PyDeleteSensor', 
                                description="delete the marker with sensorNumber in MainSystem; consistently renames sensors according to their new sensor numbers; adapts sensor numbers in sensors; items using deleted sensorNumber obtain invalid sensorNumber",
                                argList=['sensorNumber', 'suppressWarnings'],
                                defaultArgs=['','False'],
                                argTypes=['SensorIndex','bool'],
                                example = "mbs.DeleteSensor(sensorNumber=42)",
                                returnType='None',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetSensorNumber', cName='PyGetSensorNumber', 
                                description="get sensor's number by name (string)",
                                argList=['sensorName'],
                                example = "n = mbs.GetSensorNumber('test sensor')",
                                argTypes=['str'],
                                returnType='SensorIndex',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetSensor', cName='PyGetSensor', 
                                description="get sensor's dictionary by index",
                                argList=['sensorNumber'],
                                example = "sensorDict = mbs.GetSensor(0)",
                                argTypes=['SensorIndex'],
                                returnType='dict',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='ModifySensor', cName='PyModifySensor', 
                                description="modify sensor's dictionary by index",
                                argList=['sensorNumber','sensorDict'],
                                example = "mbs.ModifySensor(sensorNumber, sensorDict)",
                                argTypes=['SensorIndex','dict'],
                                returnType='None',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetSensorDefaults', cName='PyGetSensorDefaults', 
                                description="get sensor's default values for a certain sensorType as (dictionary)",
                                argList=['typeName'],
                                example = r"""sensorType = 'Node'\\sensorDict = mbs.GetSensorDefaults(sensorType)""",
                                argTypes=['str'],
                                returnType='dict',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetSensorValues', cName='PyGetSensorValues', 
                                description="get sensors's values for configuration; can be a scalar or vector-valued return value!",
                                defaultArgs=['','exu.ConfigurationType::Current'],
                                argList=['sensorNumber', 'configuration'],
                                argTypes=['SensorIndex','ConfigurationType'],
                                returnType=returnedArray,
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetSensorStoredData', cName='PyGetSensorStoredData',
                                description="get sensors's internally stored data as matrix (all time points stored); rows are containing time and sensor values as obtained by sensor (e.g., time, and x, y, and z value of position)",
                                defaultArgs=[''],
                                argList=['sensorNumber'],
                                argTypes=['SensorIndex'],
                                returnType='ArrayLike',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetSensorParameter', cName='PyGetSensorParameter', 
                                description="get sensors's parameter from sensorNumber and parameterName; parameter names can be found for the specific items in the reference manual",
                                argList=['sensorNumber', 'parameterName'],
                                argTypes=['SensorIndex','str'],
                                returnType='Any',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='SetSensorParameter', cName='PySetSensorParameter', 
                                description="set parameter 'parameterName' of sensor with sensorNumber to value; parameter names can be found for the specific items in the reference manual",
                                argList=['sensorNumber', 'parameterName', 'value'],
                                argTypes=['SensorIndex','str','Any'],
                                returnType='None',
                                )

pb.DefFinishTable() #Sensors

#now finalize pybind class, but do nothing on latex side (sL1 ignored)
pb.CppFinishClass('MainSystem') #finalize the pybind class only; nothing on the documentation side

pb.EndStubSection()
