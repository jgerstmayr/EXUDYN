#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN definition file
#
# Details:  the SystemData class.
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
pb.CreateNewRSTfile('SystemData')
pyClassStr = 'SystemData'
classStr = 'Main'+pyClassStr
pb.DefPyStartClass(classStr,pyClassStr, 
                    "A data structure of a MainSystem which mainly allows access to states and details of items (objects, nodes, loads, etc.). "+
                    "In particular access is given to system coordinates in all configurations, object and node coordinates, as well as to local-to-global (LTG) coordinate indices. "+
                    "Here, ODE2 represents second order differential equations (and coordinates), ODE1 for first order ODEs, "+
                    "AE represents algebraic equations, and Data is used for data (=history) variables that represent contact states or plastic deformation which is no classical state. "+
                    'The SystemData structure allows advanced access to this data, which HAS TO BE USED WITH CARE, as unexpected results '+
                    'and system crash might happen.',
                    labelName='sec:mbs:systemData',
                    forbidPythonConstructor=True)

# pb.AddDocu('')

pb.AddDocuCodeBlock(code="""
import exudyn as exu               #EXUDYN package including C++ core part
from exudyn.itemInterface import * #conversion of data to exudyn dictionaries
SC = exu.SystemContainer()         #container of systems
mbs = SC.AddSystem()               #add a new system to work with
nMP = mbs.AddNode(NodePoint(referenceCoordinates=[0,0,0]))
mbs.AddObject(ObjectMassPoint(physicsMass=10, nodeNumber=nMP ))
mMP = mbs.AddMarker(MarkerNodePosition(nodeNumber = nMP))
mbs.AddLoad(Force(markerNumber = mMP, loadVector=[2,0,5]))
mbs.Assemble()
mbs.SolveDynamic(exu.SimulationSettings())

#obtain current ODE2 system vector including reference values:
uTotal = mbs.systemData.GetODE2CoordinatesTotal()

#obtain current ODE2 system vector without reference values 
#  (e.g., after static simulation finished):
u = mbs.systemData.GetODE2Coordinates()
#set initial ODE2 vector for next simulation (only coordinates!):
mbs.systemData.SetODE2Coordinates(coordinates=u,
               configuration=exu.ConfigurationType.Initial)

#faster access with reference access (copy=False):
u3 = mbs.systemData.GetODE2Coordinates(copy=False)[3]
#we can also modify data, but this may be dangerous!
u3 += 1
#NOTE: reference access is possible throughout simulation and may
#      allow faster user functions, but is potentially dangerous
#      to erroneous behavior: for safety, compare with copy=True results!

#print detailed information on items:
mbs.systemData.Info()
#print LTG lists for objects and loads:
mbs.systemData.InfoLTG()
""")

pb.DefLatexStartTable(classStr)

pb.CppCode("\n//        General functions:\n")

#+++++++++++++++++++++++++++++++++
#General functions:
pb.DefPyFunctionAccess(cClass=classStr, pyName='NumberOfLoads', cName='[](const MainSystemData& msd) {return msd.GetMainLoads().NumberOfItems(); }', 
                                description="return number of loads in system",
                                isLambdaFunction = True,
                                example = 'print(mbs.systemData.NumberOfLoads())',
                                returnType='int',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='NumberOfMarkers', cName='[](const MainSystemData& msd) {return msd.GetMainMarkers().NumberOfItems(); }', 
                                description="return number of markers in system",
                                isLambdaFunction = True,
                                example = 'print(mbs.systemData.NumberOfMarkers())',
                                returnType='int',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='NumberOfNodes', cName='[](const MainSystemData& msd) {return msd.GetMainNodes().NumberOfItems(); }', 
                                description="return number of nodes in system",
                                isLambdaFunction = True,
                                example = 'print(mbs.systemData.NumberOfNodes())',
                                returnType='int',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='NumberOfObjects', cName='[](const MainSystemData& msd) {return msd.GetMainObjects().NumberOfItems(); }', 
                                description="return number of objects in system",
                                isLambdaFunction = True,
                                example = 'print(mbs.systemData.NumberOfObjects())',
                                returnType='int',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='NumberOfSensors', cName='[](const MainSystemData& msd) {return msd.GetMainSensors().NumberOfItems(); }', 
                                description="return number of sensors in system",
                                isLambdaFunction = True,
                                example = 'print(mbs.systemData.NumberOfSensors())',
                                returnType='int',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='ODE2Size', cName='PyODE2Size', 
                                description="get size of ODE2 coordinate vector for given configuration (only works correctly after mbs.Assemble() )",
                                argList=['configurationType'],
                                argTypes=['ConfigurationType'],
                                defaultArgs=['exu.ConfigurationType::Current'], #exu will be removed for binding
                                example = "print('ODE2 size=',mbs.systemData.ODE2Size())",
                                returnType='int',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='ODE1Size', cName='PyODE1Size', 
                                description="get size of ODE1 coordinate vector for given configuration (only works correctly after mbs.Assemble() )",
                                argList=['configurationType'],
                                argTypes=['ConfigurationType'],
                                defaultArgs=['exu.ConfigurationType::Current'], #exu will be removed for binding
                                example = "print('ODE1 size=',mbs.systemData.ODE1Size())",
                                returnType='int',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='AEsize', cName='PyAEsize', 
                                description="get size of AE coordinate vector for given configuration (only works correctly after mbs.Assemble() )",
                                argList=['configurationType'],
                                argTypes=['ConfigurationType'],
                                defaultArgs=['exu.ConfigurationType::Current'], #exu will be removed for binding
                                example = "print('AE size=',mbs.systemData.AEsize())",
                                returnType='int',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='DataSize', cName='PyDataSize', 
                                description="get size of Data coordinate vector for given configuration (only works correctly after mbs.Assemble() )",
                                argList=['configurationType'],
                                argTypes=['ConfigurationType'],
                                defaultArgs=['exu.ConfigurationType::Current'], #exu will be removed for binding
                                example = "print('Data size=',mbs.systemData.DataSize())",
                                returnType='int',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='SystemSize', cName='PySystemSize', 
                                description="get size of System coordinate vector for given configuration (only works correctly after mbs.Assemble() )",
                                argList=['configurationType'],
                                argTypes=['ConfigurationType'],
                                defaultArgs=['exu.ConfigurationType::Current'], #exu will be removed for binding
                                example = "print('System size=',mbs.systemData.SystemSize())",
                                returnType='int',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetTime', cName='PyGetStateTime', 
                                description="get configuration dependent time.",
                                argList=['configurationType'],
                                argTypes=['ConfigurationType'],
                                defaultArgs=['exu.ConfigurationType::Current'], #exu will be removed for binding
                                example = "mbs.systemData.GetTime(exu.ConfigurationType.Initial)",
                                returnType='float',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='SetTime', cName='PySetStateTime', 
                                description="set configuration dependent time; use this access with care, e.g., in user-defined solvers.",
                                argList=['newTime','configurationType'],
                                argTypes=['float','ConfigurationType'],
                                defaultArgs=['', 'exu.ConfigurationType::Current'], #exu will be removed for binding
                                example = "mbs.systemData.SetTime(10., exu.ConfigurationType.Initial)",
                                returnType='None',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='AddODE2LoadDependencies', cName='PyAddODE2LoadDependencies', 
                                description="advanced function for adding special dependencies of loads onto ODE2 coordinates, taking a list / numpy array of global ODE2 coordinates; this function needs to be called after Assemble() and needs to contain global ODE2 coordinate indices; this list only affects implicit or static solvers if timeIntegration.computeLoadsJacobian or staticSolver.computeLoadsJacobian is set to 1 (ODE2) or 2 (ODE2 and ODE2_t dependencies); if set, it may greatly improve convergence if loads with user functions depend on some system states, such as in a load with feedback control loop; the additional dependencies are not required, if doSystemWideDifferentiation=True, however the latter option being much less efficient. For more details, consider the file doublePendulum2DControl.py in the examples directory.",
                                argList=['loadNumber','globalODE2coordinates'],
                                defaultArgs=['',''],
                                example = "mbs.systemData.AddODE2LoadDependencies(0,[0,1,2])\\\\#add dependency of load 5 onto node 2 coordinates:\\\\nodeLTG2 = mbs.systemData.GetNodeLTGODE2(2)\\\\mbs.systemData.AddODE2LoadDependencies(5,nodeLTG2)",
                                argTypes=['float','List[int]'],
                                returnType='None',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='Info', cName='[](const MainSystemData& msd) {pout << msd.PyInfoDetailed(); }', 
                                description="print detailed information on every item; for short information use print(mbs)",
                                isLambdaFunction = True,
                                example = 'mbs.systemData.Info()',
                                returnType='None',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='InfoLTG', cName='[](const MainSystemData& msd) {pout << msd.PyInfoLTG(); }', 
                                description="print LTG information of objects and load dependencies",
                                isLambdaFunction = True,
                                example = 'mbs.systemData.InfoLTG()',
                                returnType='None',
                                )



pb.DefLatexFinishTable()

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
pb.CppCode("\n//        Coordinate access:\n")
#start new Latex/RST subsection:
pb.DefLatexStartClass(pyClassStr+': Access coordinates', '', subSection=True, labelName='sec:mbs:systemData:coordinates')

pb.AddDocu('This section provides access functions to global coordinate vectors. Assigning invalid values or using '+
            'wrong vector size might lead to system crash and unexpected results.')

pb.DefLatexStartTable(classStr+':coordinate access')
#+++++++++++++++++++++++++++++++++
#coordinate access functions:

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetODE2CoordinatesTotal', cName='GetODE2CoordsTotal', 
                                description="get ODE2 system coordinates (displacements/rotation) including reference values for given configuration (default: exu.Configuration.Current); in case of exu.ConfigurationType.Reference, it only includes reference values once and is identical to GetODE2Coordinates; note that faster access to coordinates is possibly with GetODE2Coordinates(copy=False), which is not possible with GetODE2CoordinatesTotal !",
                                argList=['configuration'],
                                argTypes=['ConfigurationType'],
                                defaultArgs=['exu.ConfigurationType::Current'],
                                example = "uTotal = mbs.systemData.GetODE2CoordinatesTotal()\\\\#this is equivalent to:\\\\uTotal=mbs.systemData.GetODE2Coordinates()+mbs.systemData.GetODE2Coordinates(exu.ConfigurationType.Reference)",
                                returnType=returnedArray,
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetODE2Coordinates', cName='GetODE2Coords', 
                                description="get ODE2 system coordinates (displacements/rotations) for given configuration (default: exu.Configuration.Current)",
                                argList=['configuration', 'copy'],
                                argTypes=['ConfigurationType', 'bool'],
                                defaultArgs=['exu.ConfigurationType::Current', 'True'],
                                example = "uCurrent = mbs.systemData.GetODE2Coordinates()",
                                returnType=returnedArray,
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='SetODE2Coordinates', cName='SetODE2Coords', 
                                description="set ODE2 system coordinates (displacements/rotations) for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!",
                                argList=['coordinates','configuration'],
                                argTypes=[listOrArray,'ConfigurationType'],
                                defaultArgs=['','exu.ConfigurationType::Current'],
                                example = "mbs.systemData.SetODE2Coordinates(uCurrent)",
                                returnType='None',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetODE2Coordinates_t', cName='GetODE2Coords_t', 
                                description="get ODE2 system coordinates (velocities) for given configuration (default: exu.Configuration.Current)",
                                argList=['configuration', 'copy'],
                                argTypes=['ConfigurationType', 'bool'],
                                defaultArgs=['exu.ConfigurationType::Current', 'True'],
                                example = "vCurrent = mbs.systemData.GetODE2Coordinates_t()",
                                returnType=returnedArray,
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='SetODE2Coordinates_t', cName='SetODE2Coords_t', 
                                description="set ODE2 system coordinates (velocities) for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!",
                                argList=['coordinates','configuration'],
                                argTypes=[listOrArray,'ConfigurationType'],
                                defaultArgs=['','exu.ConfigurationType::Current'],
                                example = "mbs.systemData.SetODE2Coordinates_t(vCurrent)",
                                returnType='None',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetODE2Coordinates_tt', cName='GetODE2Coords_tt', 
                                description="get ODE2 system coordinates (accelerations) for given configuration (default: exu.Configuration.Current)",
                                argList=['configuration', 'copy'],
                                argTypes=['ConfigurationType', 'bool'],
                                defaultArgs=['exu.ConfigurationType::Current', 'True'],
                                example = "vCurrent = mbs.systemData.GetODE2Coordinates_tt()",
                                returnType=returnedArray,
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='SetODE2Coordinates_tt', cName='SetODE2Coords_tt', 
                                description="set ODE2 system coordinates (accelerations) for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!",
                                argList=['coordinates','configuration'],
                                argTypes=[listOrArray,'ConfigurationType'],
                                defaultArgs=['','exu.ConfigurationType::Current'],
                                example = "mbs.systemData.SetODE2Coordinates_tt(aCurrent)",
                                returnType='None',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetODE1Coordinates', cName='GetODE1Coords', 
                                description="get ODE1 system coordinates (displacements) for given configuration (default: exu.Configuration.Current)",
                                argList=['configuration', 'copy'],
                                argTypes=['ConfigurationType', 'bool'],
                                defaultArgs=['exu.ConfigurationType::Current', 'True'],
                                example = "qCurrent = mbs.systemData.GetODE1Coordinates()",
                                returnType=returnedArray,
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='SetODE1Coordinates', cName='SetODE1Coords', 
                                description="set ODE1 system coordinates (velocities) for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!",
                                argList=['coordinates','configuration'],
                                argTypes=[listOrArray,'ConfigurationType'],
                                defaultArgs=['','exu.ConfigurationType::Current'],
                                example = "mbs.systemData.SetODE1Coordinates_t(qCurrent)",
                                returnType='None',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetODE1Coordinates_t', cName='GetODE1Coords_t', 
                                description="get ODE1 system coordinates (velocities) for given configuration (default: exu.Configuration.Current)",
                                argList=['configuration', 'copy'],
                                argTypes=['ConfigurationType', 'bool'],
                                defaultArgs=['exu.ConfigurationType::Current', 'True'],
                                example = "qCurrent = mbs.systemData.GetODE1Coordinates_t()",
                                returnType=returnedArray,
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='SetODE1Coordinates_t', cName='SetODE1Coords_t', 
                                description="set ODE1 system coordinates (displacements) for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!",
                                argList=['coordinates','configuration'],
                                argTypes=[listOrArray,'ConfigurationType'],
                                defaultArgs=['','exu.ConfigurationType::Current'],
                                example = "mbs.systemData.SetODE1Coordinates(qCurrent)",
                                returnType='None',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetAECoordinates', cName='GetAECoords', 
                                description="get algebraic equations (AE) system coordinates for given configuration (default: exu.Configuration.Current)",
                                argList=['configuration', 'copy'],
                                argTypes=['ConfigurationType', 'bool'],
                                defaultArgs=['exu.ConfigurationType::Current', 'True'],
                                example = "lambdaCurrent = mbs.systemData.GetAECoordinates()",
                                returnType=returnedArray,
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='SetAECoordinates', cName='SetAECoords', 
                                description="set algebraic equations (AE) system coordinates for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!",
                                argList=['coordinates','configuration'],
                                argTypes=[listOrArray,'ConfigurationType'],
                                defaultArgs=['','exu.ConfigurationType::Current'],
                                example = "mbs.systemData.SetAECoordinates(lambdaCurrent)",
                                returnType='None',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetDataCoordinates', cName='GetDataCoords', 
                                description="get system data coordinates for given configuration (default: exu.Configuration.Current)",
                                argList=['configuration', 'copy'],
                                argTypes=['ConfigurationType', 'bool'],
                                defaultArgs=['exu.ConfigurationType::Current', 'True'],
                                example = "dataCurrent = mbs.systemData.GetDataCoordinates()",
                                returnType=returnedArray,
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='SetDataCoordinates', cName='SetDataCoords', 
                                description="set system data coordinates for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!",
                                argList=['coordinates','configuration'],
                                argTypes=[listOrArray,'ConfigurationType'],
                                defaultArgs=['','exu.ConfigurationType::Current'],
                                example = "mbs.systemData.SetDataCoordinates(dataCurrent)",
                                returnType='None',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetSystemState', cName='PyGetSystemState', 
                                description="get system state for given configuration (default: exu.Configuration.Current); state vectors do not include the non-state derivatives ODE1_t and ODE2_tt and the time; function is copying data - not highly efficient; format of pyList: [ODE2Coords, ODE2Coords_t, ODE1Coords, AEcoords, dataCoords]",
                                argList=['configuration'],
                                argTypes=['ConfigurationType'],
                                defaultArgs=['exu.ConfigurationType::Current'], #exu will be removed for binding
                                example = "sysStateList = mbs.systemData.GetSystemState()",
                                returnType='List[List[float]]',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='SetSystemState', cName='PySetSystemState', 
                                description="set system data coordinates for given configuration (default: exu.Configuration.Current); invalid list of vectors / vector size may lead to system crash; write access to state vectors (but not the non-state derivatives ODE1_t and ODE2_tt and the time); function is copying data - not highly efficient; format of pyList: [ODE2Coords, ODE2Coords_t, ODE1Coords, AEcoords, dataCoords]",
                                argList=['systemStateList','configuration'],
                                argTypes=['List[List[float]]','ConfigurationType'],
                                defaultArgs=['','exu.ConfigurationType::Current'], #exu will be removed for binding
                                example = "mbs.systemData.SetSystemState(sysStateList, configuration = exu.ConfigurationType.Initial)",
                                returnType='None',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetSystemStateDict', cName='PyGetSystemStateDict', 
                                description="get dictionary with copies of (or references to) system states for given configuration (default: exu.Configuration.Current), with at least the following quantities: ODE1Coords, ODE1Coords_t, ODE2Coords, ODE2Coords_t, ODE2Coords_tt, AECoords, dataCoords; we can obtain copies OR references to vectors without copying, meaning that these vectors then have read-write properties and have to be treated carefully! The dictionary's contents are subject to changes in the future; if reference=False, data is copied",
                                argList=['configuration','reference'],
                                argTypes=['ConfigurationType','bool'],
                                defaultArgs=['exu.ConfigurationType::Current','False'], #exu will be removed for binding
                                example = "d = mbs.systemData.GetSystemStateDict()",
                                returnType='Dict[List[float]]',
                                )


pb.DefLatexFinishTable()

#+++++++++++++++++++++++++++++++++
#LTG-functions:
pb.CppCode("\n//        LTG readout functions:\n")
pb.DefLatexStartClass(pyClassStr+': Get object LTG coordinate mappings', '', subSection=True, labelName='sec:systemData:ObjectLTG')

pb.AddDocu('This section provides access functions the \\ac{LTG}-lists for every object (body, constraint, ...) '+
            'in the system. For details on the \\ac{LTG} mapping, see \\refSection{sec:overview:ltgmapping}.')

pb.DefLatexStartTable(classStr+':object LTG coordinate mappings')

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetObjectLTGODE2', cName='PyGetObjectLocalToGlobalODE2', 
                                description="get object local-to-global coordinate mapping (list of global coordinate indices) for ODE2 coordinates; only available after Assemble()",
                                argList=['objectNumber'],
                                example = "ltgObject4 = mbs.systemData.GetObjectLTGODE2(4)",
                                argTypes=['int'],
                                returnType='List[int]',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetObjectLTGODE1', cName='PyGetObjectLocalToGlobalODE1', 
                                description="get object local-to-global coordinate mapping (list of global coordinate indices) for ODE1 coordinates; only available after Assemble()",
                                argList=['objectNumber'],
                                example = "ltgObject4 = mbs.systemData.GetObjectLTGODE1(4)",
                                argTypes=['int'],
                                returnType='List[int]',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetObjectLTGAE', cName='PyGetObjectLocalToGlobalAE', 
                                description="get object local-to-global coordinate mapping (list of global coordinate indices) for algebraic equations (AE) coordinates; only available after Assemble()",
                                argList=['objectNumber'],
                                example = "ltgObject4 = mbs.systemData.GetObjectLTGAE(4)",
                                argTypes=['int'],
                                returnType='List[int]',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetObjectLTGData', cName='PyGetObjectLocalToGlobalData', 
                                description="get object local-to-global coordinate mapping (list of global coordinate indices) for data coordinates; only available after Assemble()",
                                argList=['objectNumber'],
                                example = "ltgObject4 = mbs.systemData.GetObjectLTGData(4)",
                                argTypes=['int'],
                                returnType='List[int]',
                                )

#node LTG:
pb.DefPyFunctionAccess(cClass=classStr, pyName='GetNodeLTGODE2', cName='PyGetNodeLocalToGlobalODE2', 
                                description="get node local-to-global coordinate mapping (list of global coordinate indices) for ODE2 coordinates; only available after Assemble()",
                                argList=['nodeNumber'],
                                example = "ltgNode4 = mbs.systemData.GetNodeLTGODE2(4)",
                                argTypes=['int'],
                                returnType='List[int]',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetNodeLTGODE1', cName='PyGetNodeLocalToGlobalODE1', 
                                description="get node local-to-global coordinate mapping (list of global coordinate indices) for ODE1 coordinates; only available after Assemble()",
                                argList=['nodeNumber'],
                                example = "ltgNode4 = mbs.systemData.GetNodeLTGODE1(4)",
                                argTypes=['int'],
                                returnType='List[int]',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetNodeLTGAE', cName='PyGetNodeLocalToGlobalAE', 
                                description="get node local-to-global coordinate mapping (list of global coordinate indices) for AE coordinates; only available after Assemble()",
                                argList=['nodeNumber'],
                                example = "ltgNode4 = mbs.systemData.GetNodeLTGAE(4)",
                                argTypes=['int'],
                                returnType='List[int]',
                                )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetNodeLTGData', cName='PyGetNodeLocalToGlobalData', 
                                description="get node local-to-global coordinate mapping (list of global coordinate indices) for Data coordinates; only available after Assemble()",
                                argList=['nodeNumber'],
                                example = "ltgNode4 = mbs.systemData.GetNodeLTGData(4)",
                                argTypes=['int'],
                                returnType='List[int]',
                                )


pb.DefLatexFinishTable()

#now finalize pybind class, but do nothing on latex side (sL1 ignored)
pb.CppFinishClass('SystemData') #finalize the pybind class only; nothing on the documentation side

pb.EndStubSection()
