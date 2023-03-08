# -*- coding: utf-8 -*-
"""
Created on Fri May 18 08:53:30 2018

@author: Johannes Gerstmayr

automatically generate pybindings for specific classes and functions AND latex documentation for these functions
"""

#TODO:
#add citations
#add missing figures (items, ?check other replacements)
#fix eq references as: \eq -> :eq: ...
# .. math:: e^{i\pi} + 1 = 0
#    :label: euler
#
# Euler's identity, equation :eq:`euler`, was elected one of the most
# beautiful mathematical formulas.


from autoGenerateHelper import PyLatexRST, GetDateStr #AddEnumValue, DefPyFunctionAccess, DefPyStartClass, DefPyFinishClass, DefLatexStartClass, DefLatexFinishTable
                               
import io   #for utf-8 encoding
import copy

# s = ''  #C++ pybind local includes
# sL = '' #Latex documentation
# sR = '' #RST documentation
rstList = [] #list of tuples with (file, text) => add lists, clean content (examples, lists, etc.)


#+++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++

plrmain = PyLatexRST('','', '') #plrmain will be used later, because enums come first 

plrmain.AddDocu('',section='Python-C++ command interface', sectionLevel=0, sectionLabel='sec:PCpp:command:interface')
plrmain.sRST = '' #chapter is included in CInterfaceIndex.rst

plrmain.AddDocu('This chapter lists the basic interface functions which can be used to set up a \\codeName\\ model in Python.')

plrmain.AddDocu("""This chapter lists the basic interface functions which can be used to set up 
a \\codeName\\ model in Python. Note that some functions or classes will be used in examples, which are explained in detail later on.
In the following, some basic steps and concepts for usage are shown, references to all functions are placed hereafter:
""", section='General information on Python-C++ interface', sectionLevel=1, sectionLabel='sec:generalPythonInterface')

plrmain.AddDocu('To import the module, just include the \\codeName\\ module in Python:')
plrmain.AddDocuList(itemList=['\\texttt{import exudyn as exu}'], itemText='[]')
plrmain.AddDocu('For compatibility with examples and other users, we recommend to use the \\texttt{exu} abbreviation throughout. '+
                'In addition, you may work with a convenient interface for your items, therefore also always include:')
plrmain.AddDocuList(itemList=['\\texttt{from exudyn.itemInterface import *}'], itemText='[]')
plrmain.AddDocu('Note that including \\texttt{exudyn.utilities} will cover \\texttt{itemInterface}. '+
                'Also note that \\texttt{from ... import *} is not recommended in general and it will not work in certain cases, '+
                'e.g., if you like to compute on a cluster. However, it greatly simplifies life for smaller models and you may replace '+
                'imports in your files afterwards by removing the star import.')

plrmain.AddDocu('The general hub to multibody dynamics models is provided by the classes \\texttt{SystemContainer} and \\texttt{MainSystem}, '+
                'except for some very basic system functionality (which is inside the \\codeName\\ module). \n\n'+
                'You can create a new \\texttt{SystemContainer}, which is a class that is initialized by assigning a '+
                'system container to a variable, usually denoted as \\texttt{SC}:')
plrmain.AddDocuList(itemList=['\\texttt{SC = exu.SystemContainer()}'], itemText='[]')
plrmain.AddDocu('Note that creating a second \\texttt{exu.SystemContainer()} will be independent of \\texttt{SC} and therefore usually makes no sense.\n')

plrmain.AddDocu('To add a MainSystem to system container SC and store as variable mbs, write:')
plrmain.AddDocuList(itemList=['\\texttt{mbs = SC.AddSystem()}'], itemText='[]')

plrmain.AddDocu('Furthermore, there are a couple of commands available directly in the \\texttt{exudyn} module, given in the following subsections.'+
                'Regarding the \\mybold{(basic) module access}, functions are related to the \\texttt{exudyn = exu} module, see these examples:')

plrmain.AddDocuCodeBlock(code="""
#  import exudyn module:
import exudyn as exu
#  print detailed exudyn version, Python version (at which it is compiled):
exu.GetVersionString(addDetails = True)
#  set precision of C++ output to console
exu.SetOutputPrecision(numberOfDigits)
#  turn on/off output to console
exu.SetWriteToConsole(False)
#  invalid index, may depend on compilation settings:
nInvalid = exu.InvalidIndex() #the invalid index, depends on architecture and version
""")

plrmain.AddDocu('Understanding the usage of functions for python object \\texttt{SystemContainer} of the module \\texttt{exudyn}, the following examples might help:')
plrmain.AddDocuCodeBlock(code="""
#import exudyn module:
import exudyn as exu
#  import utilities (includes itemInterface, basicUtilities, 
#                  advancedUtilities, rigidBodyUtilities, graphicsDataUtilities):
from exudyn.utilities import *
#  create system container and store in SC:
SC = exu.SystemContainer()
#  add a MainSystem (multibody system) to system container SC and store as mbs:
mbs = SC.AddSystem()
#  add a second MainSystem to system container SC and store as mbs2:
mbs2 = SC.AddSystem()
#  print number of systems available:
nSys = SC.NumberOfSystems()
exu.Print(nSys) #or just print(nSys)
#  reset system container (mbs becomes invalid):
SC.Reset()
#  delete mbs (usually not necessary):
del mbs
""")
plrmain.AddDocu('If you run a parameter variation (check \\texttt{Examples/parameterVariationExample.py}), '+
                'you may reset or delete the created \\texttt{MainSystem} \\texttt{mbs} and '+
                'the \\texttt{SystemContainer} \\texttt{SC} before creating new instances in order to avoid memory growth.')

#+++++++++++++++++++++++++++++++++++
#ITEMINDEX
plrmain.AddDocu('Many functions will work with node numbers (\\texttt{NodeIndex}), object numbers (\\texttt{ObjectIndex}),'+
                'marker numbers (\\texttt{MarkerIndex}) and others. These numbers are special objects, which have been '+
                'introduced in order to avoid mixing up, e.g., node and object numbers. \n\n'+
                'For example, the command \\texttt{mbs.AddNode(...)} returns a \\texttt{NodeIndex}. '+
                'For these indices, the following rules apply:',
                section='Item index', sectionLevel=2,sectionLabel='sec:itemIndex')
plrmain.AddDocuList(itemList=[
'\\texttt{mbs.Add[Node|Object|...](...)} returns a specific \\texttt{NodeIndex}, \\texttt{ObjectIndex}, ...',
'You can create any item index, e.g., using \\texttt{ni = NodeIndex(42)} or \\texttt{oi = ObjectIndex(42)}',
'The benefit of these indices comes as they may not be mixed up, e.g., using an object index instead of a node index.',
'You can convert any item index, e.g., NodeIndex \\texttt{ni} into an integer number using \\texttt{int(ni)} of \\texttt{ni.GetIndex()}',
'Still, you can use integers as initialization for item numbers, e.g.:\\\\\\texttt{mbs.AddObject(MassPoint(nodeNumber=13, ...))}\\\\'+
'However, it must be a pure integer type.',
'You can make integer calculations with such indices, e.g., \\texttt{oi = 2*ObjectIndex(42)+1} '+
'restricing to addition, subtraction and multiplication. Currently, the result of such calculations is a \\texttt{int} type and'+
'operating on mixed indices is not checked (but may raise exceptions in future).',
'You can also print item indices, e.g., \\texttt{print(ni)} as it converts to string by default.',
'If you are unsure about the type of an index, use \\texttt{ni.GetTypeString()} to show the index type.'
    ], itemText='[]')

plrmain.AddDocu("""As a key concept to working with \\codeName\\ , most data which is retrieved by C++ interface functions is copied.
Experienced Python users may know that it is a key concept to Python to often use references instead of copying, which is
sometimes error-prone but offers a computationally efficient behavior.
There are only a few very important cases where data is references in \\codeName\\ , the main ones are 
\\texttt{SystemContainer}, 
\\texttt{MainSystem}, 
\\texttt{VisualizationSettings}, and
\\texttt{SimulationSettings} which are always references to internal C++ classes.
The following code snippets and comments should explain this behavior:
""", section='Copying and referencing C++ objects', sectionLevel=2, sectionLabel='sec:generalPythonInterface:copyref')

plrmain.AddDocuCodeBlock(code="""
#create system container, referenced from SC:
SC = exu.SystemContainer()
SC2 = SC                           #this will only put a reference to SC
                                   #SC2 and SC represent the SAME C++ object
#add a MainSystem (multibody system):
mbs = SC.AddSystem()               #get reference mbs to C++ system
mbs2=mbs                           #again, mbs2 and mbs refer to the same C++ object
og = mbs.AddObject(ObjectGround()) #copy data of ObjectGround() into C++
o0 = mbs.GetObject(0)              #get copy of internal data as dictionary
del o0                             #delete the local dictionary; C++ data not affected
""")

#+++++++++++++++++++++++++++++++++++
#EXCEPTIONS
plrmain.AddDocu('There are several levels of type and argument checks, leading to different types of errors and exceptions. '+
                'The according error messages are non-unique, because they may be raised in Python modules or in C++, '+
                'and they may be raised on different levels of the code. Error messages depend on Python version '+
                'and on your iPython console. Very often the exception may be called \\texttt{ValueError}, but it must'+
                'not mean that it is a wrong error, but it could also be, e.g., a wrong order of function calls.',
                section='Exceptions and Error Messages', sectionLevel=2,sectionLabel='sec:cinterface:exceptions')

plrmain.AddDocu("As an example, a type conversion error is raised when providing wrong argument types, e.g., try \\texttt{exu.GetVersionString('abs')}:")

plrmain.AddDocuCodeBlock(code="""
Traceback (most recent call last):

File "C:\\Users\\username\\AppData\\Local\\Temp\\ipykernel_24988\\2212168679.py", line 1, in <module>
    exu.GetVersionString('abs')

TypeError: GetVersionString(): incompatible function arguments. The following argument types are supported:
    1. (addDetails: bool = False) -> str

Invoked with: 'abs'
""",
pythonStyle=False)

plrmain.AddDocu('Note that your particular error message may be different.')
plrmain.AddDocu('Another error results from internal type and range checking, saying User ERROR, '+
                'as it is due to a wrong input of the user. For this, we try')

plrmain.AddDocuCodeBlock(code="mbs.AddObject('abc')")

plrmain.AddDocu('Which results in a error message similar to:')
plrmain.AddDocuCodeBlock(code="""
=========================================
User ERROR [file 'C:\\Users\\username\\AppData\\Local\\Temp\\ipykernel_24988\\2838049308.py', line 1]: 
Error in AddObject(...):
Check your python code (negative indices, invalid or undefined parameters, ...)

=========================================

Traceback (most recent call last):

  File "C:\\Users\\username\\AppData\\Local\\Temp\\ipykernel_24988\\2838049308.py", line 1, in <module>
    mbs.AddObject('abc')

RuntimeError: Exudyn: parsing of Python file terminated due to Python (user) error
""", pythonStyle=False)

plrmain.AddDocu('Finally, there may be system errors. They may be caused due to previous wrong input, but '+
                'if there is no reason seen, it may be appropriate to report this error on '+
                '\\exuUrl{https://github.com/jgerstmayr/EXUDYN}{github.com/jgerstmayr/EXUDYN/} .')

plrmain.AddDocu('Be careful in reading and interpreting such error messages. You should \\mybold{read them from top to bottom}, '+
                'as the cause may be in the beginning. Often files and line numbers of errors are provided '+
                '(e.g., if you have a longer script). In the ultimate case, try to comment parts of your code or '+
                'deactivate items to see where the error comes from. See also section on Trouble shooting and FAQ.')

#+++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++
#structures and enums:


plr = PyLatexRST('','', '') #PythonLatexRST

plr.AddDocu(text='This section defines a couple of structures (C++: enum aka enumeration type), which are used to select, e.g., a configuration type or a variable type. In the background, these types are integer numbers, but for safety, the types should be used as type variables. See this examples:\n\n', 
            section='Type definitions', sectionLevel=1,sectionLabel='sec:cinterface:typedef')
#sLenum = '\section{}\n \n\n'
plr.AddDocuCodeBlock("""
#Conversion to integer is possible: 
x = int(exu.OutputVariableType.Displacement)
#also conversion from integer: 
varType = exu.OutputVariableType(8)
#use in settings:
SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.StressLocal
#use outputVariableType in sensor:
mbs.AddSensor(SensorBody(bodyNumber=rigid, storeInternal=True,
                         outputVariableType=exu.OutputVariableType.Displacement))
#
""")

plr.sPy += '\n//        pybinding to enum classes:\n'


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++
pyClass = 'OutputVariableType'

descriptionStr = 'This section shows the ' + pyClass + ' structure, which is used for selecting output values, e.g. for GetObjectOutput(...) or for selecting variables for contour plot.\n\n'
descriptionStr += 'Available output variables and the interpreation of the output variable can be found at the object definitions. \n The OutputVariableType does not provide information about the size of the output variable, which can be either scalar or a list (vector). For vector output quantities, the contour plot option offers an additional parameter for selection of the component of the OutputVariableType. The components are usually out of \{0,1,2\}, representing \{x,y,z\} components (e.g., of displacements, velocities, ...), or \{0,1,2,3,4,5\} representing \{xx,yy,zz,yz,xz,xy\} components (e.g., of strain or stress). In order to compute a norm, chose component=-1, which will result in the quadratic norm for other vectors and to a norm specified for stresses (if no norm is defined for an outputVariable, it does not compute anything)\n'

plr.sPy +=	'  py::enum_<' + pyClass + '>(m, "' + pyClass + '")\n'


plr.DefLatexStartClass(sectionName = pyClass, 
                        description=descriptionStr, 
                        subSection=True, labelName='sec:'+pyClass)
plr.DefLatexStartTable(pyClass)

#keep this list synchronized with the accoring enum structure in C++!!!
plr.AddEnumValue(pyClass, '_None', 'no value; used, e.g., to select no output variable in contour plot')
plr.AddEnumValue(pyClass, 'Distance', 'e.g., measure distance in spring damper connector')
plr.AddEnumValue(pyClass, 'Position', 'measure 3D position, e.g., of node or body')
plr.AddEnumValue(pyClass, 'Displacement', 'measure displacement; usually difference between current position and reference position')
plr.AddEnumValue(pyClass, 'DisplacementLocal', 'measure local displacement, e.g. in local joint coordinates')
plr.AddEnumValue(pyClass, 'Velocity', 'measure (translational) velocity of node or object')
plr.AddEnumValue(pyClass, 'VelocityLocal', 'measure local (translational) velocity, e.g. in local body or joint coordinates')
plr.AddEnumValue(pyClass, 'Acceleration', 'measure (translational) acceleration of node or object')
#plr.AddEnumValue(pyClass, 'AccelerationLocal', 'measure (translational) acceleration of node or object in local coordinates')

plr.AddEnumValue(pyClass, 'RotationMatrix', 'measure rotation matrix of rigid body node or object')
plr.AddEnumValue(pyClass, 'Rotation', 'measure, e.g., scalar rotation of 2D body, Euler angles of a 3D object or rotation within a joint')
plr.AddEnumValue(pyClass, 'AngularVelocity', 'measure angular velocity of node or object')
plr.AddEnumValue(pyClass, 'AngularVelocityLocal', 'measure local (body-fixed) angular velocity of node or object')
plr.AddEnumValue(pyClass, 'AngularAcceleration', 'measure angular acceleration of node or object')

plr.AddEnumValue(pyClass, 'Coordinates', 'measure the coordinates of a node or object; coordinates usually just contain displacements, but not the position values')
plr.AddEnumValue(pyClass, 'Coordinates_t', 'measure the time derivative of coordinates (= velocity coordinates) of a node or object')
plr.AddEnumValue(pyClass, 'Coordinates_tt', 'measure the second time derivative of coordinates (= acceleration coordinates) of a node or object')

plr.AddEnumValue(pyClass, 'SlidingCoordinate', 'measure sliding coordinate in sliding joint')
plr.AddEnumValue(pyClass, 'Director1', 'measure a director (e.g. of a rigid body frame), or a slope vector in local 1 or x-direction')
plr.AddEnumValue(pyClass, 'Director2', 'measure a director (e.g. of a rigid body frame), or a slope vector in local 2 or y-direction')
plr.AddEnumValue(pyClass, 'Director3', 'measure a director (e.g. of a rigid body frame), or a slope vector in local 3 or z-direction')

plr.AddEnumValue(pyClass, 'Force', 'measure global force, e.g., in joint or beam (resultant force), or generalized forces; see description of according object')
plr.AddEnumValue(pyClass, 'ForceLocal', 'measure local force, e.g., in joint or beam (resultant force)')
plr.AddEnumValue(pyClass, 'Torque', 'measure torque, e.g., in joint or beam (resultant couple/moment)')
plr.AddEnumValue(pyClass, 'TorqueLocal', 'measure local torque, e.g., in joint or beam (resultant couple/moment)')
# unused for now, maybe later on in finite elements, fluid, etc.
# plr.AddEnumValue(pyClass, 'Strain', 'measure strain, e.g., axial strain in beam')
# plr.AddEnumValue(pyClass, 'Stress', 'measure stress, e.g., axial stress in beam')
# plr.AddEnumValue(pyClass, 'Curvature', 'measure curvature; may be scalar or vectorial: twist and curvature')

plr.AddEnumValue(pyClass, 'StrainLocal', 'measure local strain, e.g., axial strain in cross section frame of beam or Green-Lagrange strain')
plr.AddEnumValue(pyClass, 'StressLocal', 'measure local stress, e.g., axial stress in cross section frame of beam or Second Piola-Kirchoff stress; choosing component==-1 will result in the computation of the Mises stress')
plr.AddEnumValue(pyClass, 'CurvatureLocal', 'measure local curvature; may be scalar or vectorial: twist and curvature of beam in cross section frame')

plr.AddEnumValue(pyClass, 'ConstraintEquation', 'evaluates constraint equation (=current deviation or drift of constraint equation)')

plr.sPy +=	'		.export_values();\n\n'
plr.DefLatexFinishTable()

#+++++++++++++++++++++++++++++++++++++++++++++++++++
pyClass = 'ConfigurationType'

descriptionStr = 'This section shows the ' + pyClass + ' structure, which is used for selecting a configuration for reading or writing information to the module. Specifically, the ConfigurationType.Current configuration is usually used at the end of a solution process, to obtain result values, or the ConfigurationType.Initial is used to set initial values for a solution process.\n\n'

plr.sPy +=	'  py::enum_<' + pyClass + '>(m, "' + pyClass + '")\n'
plr.DefLatexStartClass(sectionName = pyClass, 
                            description=descriptionStr, 
                            subSection=True, labelName='sec:'+pyClass)
plr.DefLatexStartTable(pyClass)
#keep this list synchronized with the accoring enum structure in C++!!!
plr.AddEnumValue(pyClass, '_None', 'no configuration; usually not valid, but may be used, e.g., if no configurationType is required')
plr.AddEnumValue(pyClass, 'Initial', 'initial configuration prior to static or dynamic solver; is computed during mbs.Assemble() or AssembleInitializeSystemCoordinates()')
plr.AddEnumValue(pyClass, 'Current', 'current configuration during and at the end of the computation of a step (static or dynamic)')
plr.AddEnumValue(pyClass, 'Reference', 'configuration used to define deformable bodies (reference configuration for finite elements) or joints (configuration for which some joints are defined)')
plr.AddEnumValue(pyClass, 'StartOfStep', 'during computation, this refers to the solution at the start of the step = end of last step, to which the solver falls back if convergence fails')
plr.AddEnumValue(pyClass, 'Visualization', 'this is a state completely de-coupled from computation, used for visualization')
plr.AddEnumValue(pyClass, 'EndOfEnumList', 'this marks the end of the list, usually not important to the user')

plr.sPy +=	'		.export_values();\n\n'
plr.DefLatexFinishTable()

#+++++++++++++++++++++++++++++++++++++++++++++++++++
pyClass = 'ItemType'

descriptionStr = 'This section shows the ' + pyClass + ' structure, which is used for defining types of indices, e.g., in render window and will be also used in item dictionaries in future.\n\n'

plr.sPy +=	'  py::enum_<' + pyClass + '>(m, "' + pyClass + '")\n'
plr.DefLatexStartClass(sectionName = pyClass, 
                            description=descriptionStr, 
                            subSection=True, labelName='sec:'+pyClass)
plr.DefLatexStartTable(pyClass)
#keep this list synchronized with the accoring enum structure in C++!!!
plr.AddEnumValue(pyClass, '_None', 'item has no type')
plr.AddEnumValue(pyClass, 'Node', 'item or index is of type Node')
plr.AddEnumValue(pyClass, 'Object', 'item or index is of type Object')
plr.AddEnumValue(pyClass, 'Marker', 'item or index is of type Marker')
plr.AddEnumValue(pyClass, 'Load', 'item or index is of type Load')
plr.AddEnumValue(pyClass, 'Sensor', 'item or index is of type Sensor')

plr.sPy +=	'		.export_values();\n\n'
plr.DefLatexFinishTable()

#+++++++++++++++++++++++++++++++++++++++++++++++++++
pyClass = 'NodeType'
cClass = 'Node'

descriptionStr = 'This section shows the ' + pyClass + ' structure, which is used for defining node types for 3D rigid bodies.\n\n'

plr.sPy +=	'  py::enum_<' + cClass + '::Type' + '>(m, "' + pyClass + '")\n'
plr.DefLatexStartClass(sectionName = pyClass, 
                            description=descriptionStr, 
                            subSection=True, labelName='sec:'+pyClass)
plr.DefLatexStartTable(pyClass)
#keep this list synchronized with the accoring enum structure in C++!!!
plr.AddEnumValue(cClass, '_None', 'node has no type')
plr.AddEnumValue(cClass, 'Ground', 'ground node')
plr.AddEnumValue(cClass, 'Position2D', '2D position node ')
plr.AddEnumValue(cClass, 'Orientation2D', 'node with 2D rotation')
plr.AddEnumValue(cClass, 'Point2DSlope1', '2D node with 1 slope vector')
plr.AddEnumValue(cClass, 'Position', '3D position node')
plr.AddEnumValue(cClass, 'Orientation', '3D orientation node')
plr.AddEnumValue(cClass, 'RigidBody', 'node that can be used for rigid bodies')
plr.AddEnumValue(cClass, 'RotationEulerParameters', 'node with 3D orientations that are modelled with Euler parameters (unit quaternions)')
plr.AddEnumValue(cClass, 'RotationRxyz', 'node with 3D orientations that are modelled with Tait-Bryan angles')
plr.AddEnumValue(cClass, 'RotationRotationVector', 'node with 3D orientations that are modelled with the rotation vector')
plr.AddEnumValue(cClass, 'LieGroupWithDirectUpdate', 'node to be solved with Lie group methods, without data coordinates')
plr.AddEnumValue(cClass, 'LieGroupWithDataCoordinates', 'node to be solved with Lie group methods, having data coordinates')
plr.AddEnumValue(cClass, 'GenericODE2', 'node with general ODE2 variables')
plr.AddEnumValue(cClass, 'GenericODE1', 'node with general ODE1 variables')
plr.AddEnumValue(cClass, 'GenericAE', 'node with general algebraic variables')
plr.AddEnumValue(cClass, 'GenericData', 'node with general data variables')
plr.AddEnumValue(cClass, 'Point3DSlope1', 'node with 1 slope vector')
plr.AddEnumValue(cClass, 'Point3DSlope23', 'node with 2 slope vectors in y and z direction')


plr.sPy +=	'		.export_values();\n\n'
plr.DefLatexFinishTable()

#+++++++++++++++++++++++++++++++++++++++++++++++++++
pyClass = 'JointType'
cClass = 'Joint'

descriptionStr = 'This section shows the ' + pyClass + ' structure, which is used for defining joint types, used in KinematicTree.\n\n'

plr.sPy +=	'  py::enum_<' + cClass + '::Type' + '>(m, "' + pyClass + '")\n'
plr.DefLatexStartClass(sectionName = pyClass, 
                            description=descriptionStr, 
                            subSection=True, labelName='sec:'+pyClass)
plr.DefLatexStartTable(pyClass)
#keep this list synchronized with the accoring enum structure in C++!!!
plr.AddEnumValue(cClass, '_None', 'node has no type')

plr.AddEnumValue(cClass, 'RevoluteX', 'revolute joint type with rotation around local X axis')
plr.AddEnumValue(cClass, 'RevoluteY', 'revolute joint type with rotation around local Y axis')
plr.AddEnumValue(cClass, 'RevoluteZ', 'revolute joint type with rotation around local Z axis')
plr.AddEnumValue(cClass, 'PrismaticX', 'prismatic joint type with translation along local X axis')
plr.AddEnumValue(cClass, 'PrismaticY', 'prismatic joint type with translation along local Y axis')
plr.AddEnumValue(cClass, 'PrismaticZ', 'prismatic joint type with translation along local Z axis')

plr.sPy +=	'		.export_values();\n\n'
plr.DefLatexFinishTable()

#+++++++++++++++++++++++++++++++++++++++++++++++++++
pyClass = 'DynamicSolverType'

descriptionStr = 'This section shows the ' + pyClass + ' structure, which is used for selecting dynamic solvers for simulation.\n\n'

plr.sPy +=	'  py::enum_<' + pyClass + '>(m, "' + pyClass + '")\n'
plr.DefLatexStartClass(sectionName = pyClass, 
                            description=descriptionStr, 
                            subSection=True, labelName='sec:'+pyClass)
plr.DefLatexStartTable(pyClass)
#keep this list synchronized with the accoring enum structure in C++!!!
plr.AddEnumValue(pyClass, 'GeneralizedAlpha', 'an implicit solver for index 3 problems; intended to be used for solving directly the index 3 constraints using the spectralRadius sufficiently small (usually 0.5 .. 1)')
plr.AddEnumValue(pyClass, 'TrapezoidalIndex2', 'an implicit solver for index 3 problems with index2 reduction; uses generalized alpha solver with settings for Newmark with index2 reduction')
plr.AddEnumValue(pyClass, 'ExplicitEuler',    'an explicit 1st order solver (generally not compatible with constraints)')
plr.AddEnumValue(pyClass, 'ExplicitMidpoint', 'an explicit 2nd order solver (generally not compatible with constraints)')
plr.AddEnumValue(pyClass, 'RK33',     'an explicit 3 stage 3rd order Runge-Kutta method, aka "Heun third order"; (generally not compatible with constraints)')
plr.AddEnumValue(pyClass, 'RK44',     'an explicit 4 stage 4th order Runge-Kutta method, aka "classical Runge Kutta" (generally not compatible with constraints), compatible with Lie group integration and elimination of CoordinateConstraints')
plr.AddEnumValue(pyClass, 'RK67',     "an explicit 7 stage 6th order Runge-Kutta method, see 'On Runge-Kutta Processes of High Order', J. C. Butcher, J. Austr Math Soc 4, (1964); can be used for very accurate (reference) solutions, but without step size control!")
plr.AddEnumValue(pyClass, 'ODE23',    'an explicit Runge Kutta method with automatic step size selection with 3rd order of accuracy and 2nd order error estimation, see Bogacki and Shampine, 1989; also known as ODE23 in MATLAB')
plr.AddEnumValue(pyClass, 'DOPRI5',   "an explicit Runge Kutta method with automatic step size selection with 5th order of accuracy and 4th order error estimation, see  Dormand and Prince, 'A Family of Embedded Runge-Kutta Formulae.', J. Comp. Appl. Math. 6, 1980")
plr.AddEnumValue(pyClass, 'DVERK6', '[NOT IMPLEMENTED YET] an explicit Runge Kutta solver of 6th order with 5th order error estimation; includes adaptive step selection')

plr.sPy +=	'		.export_values();\n\n'
plr.DefLatexFinishTable()

#+++++++++++++++++++++++++++++++++++++++++++++++++++
pyClass = 'KeyCode'

descriptionStr = 'This section shows the ' + pyClass + ' structure, which is used for special key codes in keyPressUserFunction.\n\n'

plr.sPy +=	'  py::enum_<' + pyClass + '>(m, "' + pyClass + '")\n'
plr.DefLatexStartClass(sectionName = pyClass, 
                            description=descriptionStr, 
                            subSection=True, labelName='sec:'+pyClass)
plr.DefLatexStartTable(pyClass)
#keep this list synchronized with the accoring enum structure in C++!!!
plr.AddEnumValue(pyClass, 'SPACE', 'space key')
plr.AddEnumValue(pyClass, 'ENTER', 'enter (return) key')
plr.AddEnumValue(pyClass, 'TAB',   '')
plr.AddEnumValue(pyClass, 'BACKSPACE', '')
plr.AddEnumValue(pyClass, 'RIGHT', 'cursor right')
plr.AddEnumValue(pyClass, 'LEFT', 'cursor left')
plr.AddEnumValue(pyClass, 'DOWN', 'cursor down')
plr.AddEnumValue(pyClass, 'UP', 'cursor up')
plr.AddEnumValue(pyClass, 'F1', 'function key F1')
plr.AddEnumValue(pyClass, 'F2', 'function key F2')
plr.AddEnumValue(pyClass, 'F3', 'function key F3')
plr.AddEnumValue(pyClass, 'F4', 'function key F4')
plr.AddEnumValue(pyClass, 'F5', 'function key F5')
plr.AddEnumValue(pyClass, 'F6', 'function key F6')
plr.AddEnumValue(pyClass, 'F7', 'function key F7')
plr.AddEnumValue(pyClass, 'F8', 'function key F8')
plr.AddEnumValue(pyClass, 'F9', 'function key F9')
plr.AddEnumValue(pyClass, 'F10', 'function key F10')

plr.sPy +=	'		.export_values();\n\n'
plr.DefLatexFinishTable()

#+++++++++++++++++++++++++++++++++++++++++++++++++++
pyClass = 'LinearSolverType'


descriptionStr = 'This section shows the ' + pyClass + ' structure, which is used for selecting linear solver types, which are dense or sparse solvers.\n\n'

plr.sPy +=	'  py::enum_<' + pyClass + '>(m, "' + pyClass + '")\n'
plr.DefLatexStartClass(sectionName = pyClass, 
                            description=descriptionStr, 
                            subSection=True, labelName='sec:'+pyClass)
plr.DefLatexStartTable(pyClass)
#keep this list synchronized with the accoring enum structure in C++!!!
plr.AddEnumValue(pyClass, '_None', 'no value; used, e.g., if no solver is selected')
plr.AddEnumValue(pyClass, 'EXUdense', 'use dense matrices and according solvers for densly populated matrices (usually the CPU time grows cubically with the number of unknowns)')
plr.AddEnumValue(pyClass, 'EigenSparse', 'use sparse matrices and according solvers; additional overhead for very small multibody systems; specifically, memory allocation is performed during a factorization process')
plr.AddEnumValue(pyClass, 'EigenSparseSymmetric', 'use sparse matrices and according solvers; NOTE: this is the symmetric mode, which assumes symmetric system matrices; this is EXPERIMENTAL and should only be used of user knows that the system matrices are (nearly) symmetric; does not work with scaled GeneralizedAlpha matrices; does not work with constraints, as it must be symmetric positive definite')

plr.sPy +=	'		.export_values();\n\n'
plr.DefLatexFinishTable()


#+++++++++++++++++++++++++++++++++++++++++++++++++++
pyClass = 'ContactTypeIndex'
cClass = 'Contact'

descriptionStr = 'This section shows the ' + pyClass + ' structure, which is in GeneralContact to select specific contact items, such as spheres, ANCFCable or triangle items.\n\n'

plr.sPy +=	'  py::enum_<' + cClass + '::TypeIndex' + '>(m, "' + pyClass + '")\n'
plr.DefLatexStartClass(sectionName = pyClass, 
                            description=descriptionStr, 
                            subSection=True, labelName='sec:'+pyClass)
plr.DefLatexStartTable(pyClass)
#keep this list synchronized with the accoring enum structure in C++!!!
plr.AddEnumValue(cClass, 'IndexSpheresMarkerBased', 'spheres attached to markers')
plr.AddEnumValue(cClass, 'IndexANCFCable2D', 'ANCFCable2D contact items')
plr.AddEnumValue(cClass, 'IndexTrigsRigidBodyBased', 'triangles attached to rigid body (or rigid body marker)')
plr.AddEnumValue(cClass, 'IndexEndOfEnumList', 'signals end of list')

plr.sPy +=	'		.export_values();\n\n'
plr.DefLatexFinishTable()

sLenum = plr.sLatex #the latex enum string is used later on!
sRSTenum = plr.sRST #the latex enum string is used later on!

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#now start the main file:
plr.sRST = ''
plr.sLatex = ''
plr.CreateNewRSTfile('GeneralInformation')

plr.sLatex = plrmain.sLatex
plr.sRST = plrmain.sRST


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#Access functions to EXUDYN
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
plr.CreateNewRSTfile('Exudyn')
plr.DefPyStartClass('','', '')

plr.AddDocu('These are the access functions to the \\codeName\\ module. General usage is explained '+
                'in \\refSection{sec:generalPythonInterface} and examples are provided there. '+
                'The C++ module \\texttt{exudyn} is the root level object linked between Python and C++.'+
                'In the installed site-packages, the according file is usually denoted as \\texttt{exudynCPP.pyd} for the regular module, '+
                '\\texttt{exudynCPPfast.pyd} for the module without range checks and \\texttt{exudynCPPnoAVX.pyd} '+
                'for the module compiled without AVX vector extensions (may depend on your installation).')

plr.AddDocuCodeBlock(code="""
#import exudyn module:
import exudyn as exu
#create systemcontainer and mbs:
SC = exu.SystemContainer()
mbs = SC.AddSystem()
""")
plr.DefLatexStartTable('exudyn')

plr.DefPyFunctionAccess('', 'GetVersionString', 'PyGetVersionString', 
                               argList=['addDetails'],
                               defaultArgs=['False'],
                               description='Get Exudyn built version as string (if addDetails=True, adds more information on compilation Python version, platform, etc.; the Python micro version may differ from that you are working with; AVX2 shows that you are running a AVX2 compiled version)',
                               )

plr.DefPyFunctionAccess('', 'Help', 'PyHelp', 
                               description='Show basic help information',
                               )

sOld = plr.PyStr()
plr.DefPyFunctionAccess('', 'RequireVersion', '', 
                               argList=['requiredVersionString'],
                               description = 'Checks if the installed version is according to the required version. Major, micro and minor version must agree the required level. This function is defined in the \\texttt{__init__.py} file', 
                               example='exu.RequireVersion("1.0.31")')
plr.sPy = sOld #this function is defined in __init__.py ==> do not add to cpp bindings

plr.DefPyFunctionAccess(cClass='', pyName='StartRenderer', cName='PyStartOpenGLRenderer', 
                                defaultArgs=['0'],
                                argList=['verbose'],
                                description="Start OpenGL rendering engine (in separate thread) for visualization of rigid or flexible multibody system; use verbose=1 to output information during OpenGL window creation; verbose=2 produces more output and verbose=3 gives a debug level; some of the information will only be seen in windows command (powershell) windows or linux shell, but not inside iPython of e.g. Spyder")

#new, defined in C++ as lambda function:
sOld = plr.PyStr()
plr.DefPyFunctionAccess('', 'StopRenderer', 'no direct link to C++ here', "Stop OpenGL rendering engine")
plr.sPy = sOld

plr.DefPyFunctionAccess(cClass='', pyName='IsRendererActive', cName='PyIsRendererActive', 
                                description="returns True if GLFW renderer is available and running; otherwise False")

plr.DefPyFunctionAccess(cClass='', pyName='DoRendererIdleTasks', cName='PyDoRendererIdleTasks', 
                                defaultArgs=['0'],
                                argList=['waitSeconds'],
                                description="Call this function in order to interact with Renderer window; use waitSeconds in order to run this idle tasks while animating a model (e.g. waitSeconds=0.04), use waitSeconds=0 without waiting, or use waitSeconds=-1 to wait until window is closed")

sOld = plr.PyStr()
plr.DefPyFunctionAccess(cClass='', pyName='SolveStatic', cName='SolveDynamic', 
                               description='Static solver function, mapped from module \\texttt{solver}, to solve static equations (without inertia terms) of constrained rigid or flexible multibody system; for details on the Python interface see \\refSection{sec:solver:SolveStatic}; for background on solvers, see \\refSection{sec:solvers}',
                               argList=['mbs', 'simulationSettings', 'updateInitialValues', 'storeSolver'],
                               defaultArgs=['','exudyn.SimulationSettings()','False','True']
                               )
                
plr.DefPyFunctionAccess(cClass='', pyName='SolveDynamic', cName='SolveDynamic', 
                               description='Dynamic solver function, mapped from module \\texttt{solver}, to solve equations of motion of constrained rigid or flexible multibody system; for details on the Python interface see \\refSection{sec:solver:SolveDynamic}; for background on solvers, see \\refSection{sec:solvers}',
                               argList=['mbs', 'simulationSettings', 'solverType', 'updateInitialValues', 'storeSolver'],
                               defaultArgs=['','exudyn.SimulationSettings()','exudyn.DynamicSolverType.GeneralizedAlpha','False','True']
                               )
                
plr.DefPyFunctionAccess(cClass='', pyName='ComputeODE2Eigenvalues', cName='ComputeODE2Eigenvalues', 
                               description='Simple interface to scipy eigenvalue solver for eigenvalue analysis of the second order differential equations part in mbs, mapped from module \\texttt{solver}; for details on the Python interface see \\refSection{sec:solver:ComputeODE2Eigenvalues}',
                               argList=['mbs', 'simulationSettings', 'useSparseSolver', 'numberOfEigenvalues', 'setInitialValues', 'convert2Frequencies'],
                               defaultArgs=['','exudyn.SimulationSettings()','False','-1','True','False'])
plr.sPy = sOld

plr.DefPyFunctionAccess(cClass='', pyName='SetOutputPrecision', cName='PySetOutputPrecision', 
                                description="Set the precision (integer) for floating point numbers written to console (reset when simulation is started!); NOTE: this affects only floats converted to strings inside C++ exudyn; if you print a float from Python, it is usually printed with 16 digits; if printing numpy arrays, 8 digits are used as standard, to be changed with numpy.set_printoptions(precision=16); alternatively convert into a list",
                                argList=['numberOfDigits'])

plr.DefPyFunctionAccess(cClass='', pyName='SetLinalgOutputFormatPython', cName='PySetLinalgOutputFormatPython', 
                                description="True: use Python format for output of vectors and matrices; False: use matlab format",
                                argList=['flagPythonFormat'])

plr.DefPyFunctionAccess(cClass='', pyName='SetWriteToConsole', cName='PySetWriteToConsole', 
                            description="set flag to write (True) or not write to console; default = True",
                            argList=['flag'])

plr.DefPyFunctionAccess(cClass='', pyName='SetWriteToFile', cName='PySetWriteToFile', 
                            description="set flag to write (True) or not write to console; default value of flagWriteToFile = False; flagAppend appends output to file, if set True; in order to finalize the file, write \\texttt{exu.SetWriteToFile('', False)} to close the output file",
                            argList=['filename', 'flagWriteToFile', 'flagAppend'],
                            defaultArgs=['', 'True', 'False'],
                            example="exu.SetWriteToConsole(False) \\#no output to console\\\\exu.SetWriteToFile(filename='testOutput.log', flagWriteToFile=True, flagAppend=False)\\\\exu.Print('print this to file')\\\\exu.SetWriteToFile('', False) \\#terminate writing to file which closes the file"
                            )

plr.DefPyFunctionAccess(cClass='', pyName='SetPrintDelayMilliSeconds', cName='PySetPrintDelayMilliSeconds', 
                            description="add some delay (in milliSeconds) to printing to console, in order to let Spyder process the output; default = 0",
                            argList=['delayMilliSeconds'])

plr.DefPyFunctionAccess(cClass='', pyName='Print', cName='PyPrint', 
                            description="this allows printing via exudyn with similar syntax as in Python print(args) except for keyword arguments: print('test=',42); allows to redirect all output to file given by SetWriteToFile(...); does not output in case that SetWriteToConsole is set to False",
                            #argList=['pyObject'] #not compatible with py::args
                            )

plr.DefPyFunctionAccess(cClass='', pyName='SuppressWarnings', cName='PySuppressWarnings', 
                            description="set flag to suppress (=True) or enable (=False) warnings",
                            argList=['flag'])

plr.DefPyFunctionAccess(cClass='', pyName='InfoStat', cName='PythonInfoStat', 
                               description='Retrieve list of global information on memory allocation and other counts as list:[array_new_counts, array_delete_counts, vector_new_counts, vector_delete_counts, matrix_new_counts, matrix_delete_counts, linkedDataVectorCast_counts]; May be extended in future; if writeOutput==True, it additionally prints the statistics; counts for new vectors and matrices should not depend on numberOfSteps, except for some objects such as ObjectGenericODE2 and for (sensor) output to files; Not available if code is compiled with __FAST_EXUDYN_LINALG flag',
                               argList=['writeOutput'],
                               defaultArgs=['True'])

plr.DefPyFunctionAccess('', 'Go', 'PythonGo', 'Creates a SystemContainer SC and a main multibody system mbs')

plr.DefPyFunctionAccess('', 'InvalidIndex', 'GetInvalidIndex', 
                            "This function provides the invalid index, which may depend on the kind of 32-bit, 64-bit signed or unsigned integer; e.g. node index or item index in list; currently, the InvalidIndex() gives -1, but it may be changed in future versions, therefore you should use this function")

#plr.sPy += '        m.def_readwrite("variables", &exudynVariables, py::return_value_policy::reference)\n' 
#variables in the module itself are exported with "m.attr(...)" !
plr.sPy += '        m.attr("variables") = exudynVariables;\n' 
plr.DefLatexDataAccess('variables','this dictionary may be used by the user to store exudyn-wide data in order to avoid global Python variables; usage: exu.variables["myvar"] = 42 ')

plr.sPy += '        m.attr("sys") = exudynSystemVariables;\n' 
plr.DefLatexDataAccess('sys',"this dictionary is used and reserved by the system, e.g. for testsuite, graphics or system function to store module-wide data in order to avoid global Python variables; the variable exu.sys['renderState'] contains the last render state after exu.StopRenderer() and can be used for subsequent simulations ")

plr.DefPyFinishClass('')






#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#currently, only latex binding:
plr.CreateNewRSTfile('SystemContainer')
pyClassStr = 'SystemContainer'
classStr = 'Main'+pyClassStr
sPyOld = plr.PyStr() #systemcontainer manually added in C++

plr.DefPyStartClass(classStr, pyClassStr, '')

plr.AddDocu('The SystemContainer is the top level of structures in \\codeName. '+
                'The container holds all (multibody) systems, solvers and all other data structures for computation. '+
                'Currently, only one container shall be used. In future, multiple containers might be usable at the same time.'+
                'Regarding the \\mybold{(basic) module access}, functions are related to the \\texttt{exudyn = exu} module, '+
                'see also the introduction of this chapter and this example:')

plr.AddDocuCodeBlock(code="""
import exudyn as exu
#create system container and store by reference in SC:
SC = exu.SystemContainer() 
#add MainSystem to SC:
mbs = SC.AddSystem()
""")

plr.DefLatexStartTable(pyClassStr)

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#GENERAL FUNCTIONS

plr.DefPyFunctionAccess(cClass=classStr, pyName='Reset', cName='Reset', 
                                description="delete all multibody systems and reset SystemContainer (including graphics); this also releases SystemContainer from the renderer, which requires SC.AttachToRenderEngine() to be called in order to reconnect to rendering; a safer way is to delete the current SystemContainer and create a new one (SC=SystemContainer() )")

plr.DefPyFunctionAccess(cClass=classStr, pyName='AddSystem', cName='AddMainSystem', 
                                description="add a new computational system", options='py::return_value_policy::reference')

plr.DefPyFunctionAccess(cClass=classStr, pyName='NumberOfSystems', cName='NumberOfSystems', 
                                description="obtain number of multibody systems available in system container")

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetSystem', cName='GetMainSystem', 
                                description="obtain multibody systems with index from system container",
                                argList=['systemNumber'])


#plr.sPy += '        .def_property("visualizationSettings", &MainSystemContainer::PyGetVisualizationSettings, &MainSystemContainer::PySetVisualizationSettings)\n' 
plr.DefLatexDataAccess('visualizationSettings','this structure is read/writeable and contains visualization settings, which are immediately applied to the rendering window. \\tabnewline\n    EXAMPLE:\\tabnewline\n    SC = exu.SystemContainer()\\tabnewline\n    SC.visualizationSettings.autoFitScene=False  ')

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetRenderState', cName='PyGetRenderState', 
                                description="Get dictionary with current render state (openGL zoom, modelview, etc.); will have no effect if GLFW_GRAPHICS is deactivated",
                                example = "SC = exu.SystemContainer()\\\\renderState = SC.GetRenderState() \\\\print(renderState['zoom'])"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetRenderState', cName='PySetRenderState', 
                                description="Set current render state (openGL zoom, modelview, etc.) with given dictionary; usually, this dictionary has been obtained with GetRenderState; will have no effect if GLFW_GRAPHICS is deactivated",
                                example = "SC = exu.SystemContainer()\\\\SC.SetRenderState(renderState)",
                                argList=['renderState'],
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='RedrawAndSaveImage', cName='RedrawAndSaveImage', 
                                description="Redraw openGL scene and save image (command waits until process is finished)")

plr.DefPyFunctionAccess(cClass=classStr, pyName='WaitForRenderEngineStopFlag', cName='WaitForRenderEngineStopFlag', 
                                description="Wait for user to stop render engine (Press 'Q' or Escape-key); this command is used to have active response of the render window, e.g., to open the visualization dialog or use the right-mouse-button; behaves similar as mbs.WaitForUserToContinue()")

plr.DefPyFunctionAccess(cClass=classStr, pyName='RenderEngineZoomAll', cName='PyZoomAll', 
                                description="Send zoom all signal, which will perform zoom all at next redraw request")

plr.DefPyFunctionAccess(cClass=classStr, pyName='RedrawAndSaveImage', cName='RedrawAndSaveImage', 
                                description="Redraw openGL scene and save image (command waits until process is finished)")

plr.DefPyFunctionAccess(cClass=classStr, pyName='AttachToRenderEngine', cName='AttachToRenderEngine', 
                                description="Links the SystemContainer to the render engine, such that the changes in the graphics structure drawn upon updates, etc.; done automatically on creation of SystemContainer; return False, if no renderer exists (e.g., compiled without GLFW) or cannot be linked (if other SystemContainer already linked)")

plr.DefPyFunctionAccess(cClass=classStr, pyName='DetachFromRenderEngine', cName='DetachFromRenderEngine', 
                                description="Releases the SystemContainer from the render engine; return True if successfully released, False if no GLFW available or detaching failed")

plr.DefPyFunctionAccess(cClass=classStr, pyName='SendRedrawSignal', cName='SendRedrawSignal', 
                                description="This function is used to send a signal to the renderer that all MainSystems (mbs) shall be redrawn")

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetCurrentMouseCoordinates', cName='PyGetCurrentMouseCoordinates', 
                                description="Get current mouse coordinates as list [x, y]; x and y being floats, as returned by GLFW, measured from top left corner of window; use GetCurrentMouseCoordinates(useOpenGLcoordinates=True) to obtain OpenGLcoordinates of projected plane",
                                argList=['useOpenGLcoordinates'],
                                defaultArgs=['False'],
                                )
plr.sPy = sPyOld  #system container manually added 

#removed 2021-07-12 as deprecated:
# plr.DefPyFunctionAccess(cClass=classStr, pyName='TimeIntegrationSolve', cName="""[](MainSystemContainer& msc, MainSystem& ms, HString solverName, const SimulationSettings& simulationSettings) {
#                             		pout.precision(simulationSettings.outputPrecision);
#                             		if (solverName == "RungeKutta1")
#                             			msc.GetSolvers().GetSolverRK1().SolveSystem(simulationSettings, *(ms.GetCSystem()));
#                             		else if (solverName == "GeneralizedAlpha")
#                             			msc.GetSolvers().GetSolverGeneralizedAlpha().SolveSystem(simulationSettings, *(ms.GetCSystem()));
#                             		else
#                             			PyError(HString("SystemContainer::TimeIntegrationSolve: invalid solverName '")+solverName+"'; options are: RungeKutta1 or GeneralizedAlpha");
#                             		}""", 
#                                 argList=['mainSystem','solverName','simulationSettings'],
#                                 description="DEPRECATED, use exu.SolveDynamic(...) instead, see \\refSection{sec:solver:SolveDynamic}! Call time integration solver for given system with solverName ('RungeKutta1'...explicit solver, 'GeneralizedAlpha'...implicit solver); use simulationSettings to individually configure the solver",
#                                 example = "simSettings = exu.SimulationSettings()\\\\simSettings.timeIntegration.numberOfSteps = 1000\\\\simSettings.timeIntegration.endTime = 2\\\\simSettings.timeIntegration.verboseMode = 1\\\\SC.TimeIntegrationSolve(mbs, 'GeneralizedAlpha', simSettings)",
#                                 isLambdaFunction = True
#                                 )

# plr.DefPyFunctionAccess(cClass=classStr, pyName='StaticSolve', cName="""[](MainSystemContainer& msc, MainSystem& ms, const SimulationSettings& simulationSettings) {
#                                 pout.precision(simulationSettings.outputPrecision);
#                                 msc.GetSolvers().GetSolverStatic().SolveSystem(simulationSettings, *(ms.GetCSystem()));
#                                 }""", 
#                                 argList=['mainSystem','simulationSettings'],
#                                 description="DEPRECATED, use exu.SolveStatic(...) instead, see \\refSection{sec:solver:SolveStatic}! Call solver to compute a static solution of the system, considering acceleration and velocity coordinates to be zero (initial velocities may be considered by certain objects)",
#                                 example = "simSettings = exu.SimulationSettings()\\\\simSettings.staticSolver.newton.relativeTolerance = 1e-6\\\\SC.StaticSolve(mbs, simSettings)",
#                                 isLambdaFunction = True
#                                 )


plr.DefLatexFinishTable()#only finalize latex table



#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
plr.CreateNewRSTfile('MainSystem')
classStr = 'MainSystem'
plr.DefPyStartClass(classStr, classStr, '')

plr.AddDocu("This is the structure which defines a (multibody) system. "+
            "In C++, there is a MainSystem (links to Python) and a System (computational part). "+
            "For that reason, the name is MainSystem on the Python side, but it is often just called 'system'. "+
            "For compatibility, it is recommended to denote the variable holding this system as mbs, the multibody dynamics system. "+
            "It can be created, visualized and computed. Use the following functions for system manipulation.")

plr.AddDocuCodeBlock(code="""
import exudyn as exu
SC = exu.SystemContainer()
mbs = SC.AddSystem()
""")

plr.DefLatexStartTable(classStr)

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#GENERAL FUNCTIONS

plr.DefPyFunctionAccess(cClass=classStr, pyName='Assemble', cName='Assemble', 
                                description="assemble items (nodes, bodies, markers, loads, ...) of multibody system; Calls CheckSystemIntegrity(...), AssembleCoordinates(), AssembleLTGLists(), AssembleInitializeSystemCoordinates(), and AssembleSystemInitialize()")

plr.DefPyFunctionAccess(cClass=classStr, pyName='AssembleCoordinates', cName='AssembleCoordinates', 
                                description="assemble coordinates: assign computational coordinates to nodes and constraints (algebraic variables)")

plr.DefPyFunctionAccess(cClass=classStr, pyName='AssembleLTGLists', cName='AssembleLTGLists', 
                                description="build \\ac{LTG} coordinate lists for objects (used to build global ODE2RHS, MassMatrix, etc. vectors and matrices) and store special object lists (body, connector, constraint, ...)")

plr.DefPyFunctionAccess(cClass=classStr, pyName='AssembleInitializeSystemCoordinates', cName='AssembleInitializeSystemCoordinates', 
                                description="initialize all system-wide coordinates based on initial values given in nodes")

plr.DefPyFunctionAccess(cClass=classStr, pyName='AssembleSystemInitialize', cName='AssembleSystemInitialize', 
                                description="initialize some system data, e.g., generalContact objects (searchTree, etc.)")

plr.DefPyFunctionAccess(cClass=classStr, pyName='Reset', cName='Reset', 
                                description="reset all lists of items (nodes, bodies, markers, loads, ...) and temporary vectors; deallocate memory")

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetSystemContainer', cName='GetMainSystemContainer', 
                                description="return the systemContainer where the mainSystem (mbs) was created")

plr.DefPyFunctionAccess(cClass=classStr, pyName='WaitForUserToContinue', cName='WaitForUserToContinue', 
                                argList=['printMessage'],
                                defaultArgs=['True'],
                                description="interrupt further computation until user input --> 'pause' function; this command runs a loop in the background to have active response of the render window, e.g., to open the visualization dialog or use the right-mouse-button; behaves similar as SC.WaitForRenderEngineStopFlagthis()")

plr.DefPyFunctionAccess(cClass=classStr, pyName='SendRedrawSignal', cName='SendRedrawSignal', 
                                description="this function is used to send a signal to the renderer that the scene shall be redrawn because the visualization state has been updated")

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetRenderEngineStopFlag', cName='GetRenderEngineStopFlag', 
                                description="get the current stop simulation flag; True=user wants to stop simulation")

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetRenderEngineStopFlag', cName='SetRenderEngineStopFlag', 
                                description="set the current stop simulation flag; set to False, in order to continue a previously user-interrupted simulation")

plr.DefPyFunctionAccess(cClass=classStr, pyName='ActivateRendering', cName='ActivateRendering', 
                                argList=['flag'],
                                defaultArgs=['True'],
                                description="activate (flag=True) or deactivate (flag=False) rendering for this system")

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetPreStepUserFunction', cName='PySetPreStepUserFunction', 
                                description="Sets a user function PreStepUserFunction(mbs, t) executed at beginning of every computation step; in normal case return True; return False to stop simulation after current step",
                                example = 'def PreStepUserFunction(mbs, t):\\\\ \\TAB print(mbs.systemData.NumberOfNodes())\\\\ \\TAB if(t>1): \\\\ \\TAB  \\TAB return False \\\\ \\TAB return True \\\\ mbs.SetPreStepUserFunction(PreStepUserFunction)')
                                                      
plr.DefPyFunctionAccess(cClass=classStr, pyName='SetPostNewtonUserFunction', cName='PySetPostNewtonUserFunction', 
                                description="Sets a user function PostNewtonUserFunction(mbs, t) executed after successful Newton iteration in implicit or static solvers and after step update of explicit solvers, but BEFORE PostNewton functions are called by the solver; function returns list [discontinuousError, recommendedStepSize], containing a error of the PostNewtonStep, which is compared to [solver].discontinuous.iterationTolerance. The recommendedStepSize shall be negative, if no recommendation is given, 0 in order to enforce minimum step size or a specific value to which the current step size will be reduced and the step will be repeated; use this function, e.g., to reduce step size after impact or change of data variables",
                                example = 'def PostNewtonUserFunction(mbs, t):\\\\ \\TAB if(t>1): \\\\ \\TAB  \\TAB return [0, 1e-6] \\\\ \\TAB return [0,0] \\\\ mbs.SetPostNewtonUserFunction(PostNewtonUserFunction)')

#contact:                                      
plr.DefPyFunctionAccess(cClass=classStr, pyName='AddGeneralContact', cName='AddGeneralContact', 
                                description="add a new general contact, used to enable efficient contact computation between objects (nodes or markers)", 
                                options='py::return_value_policy::reference')

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetGeneralContact', cName='GetGeneralContact', 
                                description="get read/write access to GeneralContact with index generalContactNumber stored in mbs; Examples shows how to access the GeneralContact object added with last AddGeneralContact() command:",
                                example = 'gc=mbs.GetGeneralContact(mbs.NumberOfGeneralContacts()-1)',
                                argList=['generalContactNumber'])

plr.DefPyFunctionAccess(cClass=classStr, pyName='DeleteGeneralContact', cName='DeleteGeneralContact', 
                                description="delete GeneralContact with index generalContactNumber in mbs; other general contacts are resorted (index changes!)",
                                argList=['generalContactNumber'])

plr.DefPyFunctionAccess(cClass=classStr, pyName='NumberOfGeneralContacts', cName='NumberOfGeneralContacts', 
                                description="Return number of GeneralContact objects in mbs", 
                                )
#++++++++++++++++

#old version, with variables: plr.DefPyFunctionAccess(cClass=classStr, pyName='__repr__', cName='[](const MainSystem &ms) {\n            return "<systemData: \\n" + ms.GetMainSystemData().PyInfoSummary() + "\\nmainSystem:\\n  variables = " + EXUstd::ToString(ms.variables) + "\\n  sys = " + EXUstd::ToString(ms.systemVariables) + "\\n>\\n"; }', 
plr.DefPyFunctionAccess(cClass=classStr, pyName='__repr__', cName='[](const MainSystem &ms) {\n            return "<systemData: \\n" + ms.GetMainSystemData().PyInfoSummary() + "\\nFor details see mbs.systemData, mbs.sys and mbs.variables\\n>\\n"; }', 
                                description="return the representation of the system, which can be, e.g., printed",
                                isLambdaFunction = True,
                                example = 'print(mbs)')

plr.sPy += '        .def_property("systemIsConsistent", &MainSystem::GetFlagSystemIsConsistent, &MainSystem::SetFlagSystemIsConsistent)\n' 
plr.DefLatexDataAccess('systemIsConsistent','this flag is used by solvers to decide, whether the system is in a solvable state; this flag is set to False as long as Assemble() has not been called; any modification to the system, such as Add...(), Modify...(), etc. will set the flag to False again; this flag can be modified (set to True), if a change of e.g.~an object (change of stiffness) or load (change of force) keeps the system consistent, but would normally lead to systemIsConsistent=False')

plr.sPy += '        .def_property("interactiveMode", &MainSystem::GetInteractiveMode, &MainSystem::SetInteractiveMode)\n' 
plr.DefLatexDataAccess('interactiveMode','set this flag to True in order to invoke a Assemble() command in every system modification, e.g. AddNode, AddObject, ModifyNode, ...; this helps that the system can be visualized in interactive mode.')

plr.sPy += '        .def_readwrite("variables", &MainSystem::variables, py::return_value_policy::reference)\n' 
plr.DefLatexDataAccess('variables','this dictionary may be used by the user to store model-specific data, in order to avoid global Python variables in complex models; mbs.variables["myvar"] = 42 ')

plr.sPy += '        .def_readwrite("sys", &MainSystem::systemVariables, py::return_value_policy::reference)\n' 
plr.DefLatexDataAccess('sys','this dictionary is used by exudyn Python libraries, e.g., solvers, to avoid global Python variables ')

plr.sPy += '        .def_property("solverSignalJacobianUpdate", &MainSystem::GetFlagSolverSignalJacobianUpdate, &MainSystem::SetFlagSolverSignalJacobianUpdate)\n' 
plr.DefLatexDataAccess('solverSignalJacobianUpdate','this flag is used by solvers to decide, whether the jacobian should be updated; at beginning of simulation and after jacobian computation, this flag is set automatically to False; use this flag to indicate system changes, e.g. during time integration  ')

plr.sPy += '        .def_readwrite("systemData", &MainSystem::mainSystemData, py::return_value_policy::reference)\n' 
plr.DefLatexDataAccess('systemData','Access to SystemData structure; enables access to number of nodes, objects, ... and to (current, initial, reference, ...) state variables (ODE2, AE, Data,...)')

plr.DefLatexFinishTable()#only finalize latex table

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#NODE
plr.sPy += "\n//        NODES:\n"
plr.DefLatexStartClass(classStr+': Node','', subSection=True,labelName='sec:mainsystem:node')

plr.AddDocu('This section provides functions for adding, reading and modifying nodes. '+
            'Nodes are used to define coordinates (unknowns to the static system and degrees of freedom '+
            'if constraints are not present). Nodes can provide various types of coordinates for '+
            'second/first order differential equations (ODE2/ODE1), algebraic equations (AE) and for data '+
            '(history) variables -- which are not providing unknowns in the nonlinear solver but will be solved '+
            'in an additional nonlinear iteration for e.g. contact, friction or plasticity.')

plr.AddDocuCodeBlock(code="""
import exudyn as exu               #EXUDYN package including C++ core part
from exudyn.itemInterface import * #conversion of data to exudyn dictionaries
SC = exu.SystemContainer()         #container of systems
mbs = SC.AddSystem()               #add a new system to work with
nMP = mbs.AddNode(NodePoint2D(referenceCoordinates=[0,0]))
""")

plr.DefLatexStartTable(classStr+':nodes')

plr.DefPyFunctionAccess(cClass=classStr, pyName='AddNode', cName='AddMainNodePyClass', 
                                description="add a node with nodeDefinition from Python node class; returns (global) node index (type NodeIndex) of newly added node; use int(nodeIndex) to convert to int, if needed (but not recommended in order not to mix up index types of nodes, objects, markers, ...)",
                                argList=['pyObject'],
                                example = "item = Rigid2D( referenceCoordinates= [1,0.5,0], initialVelocities= [10,0,0]) \\\\mbs.AddNode(item) \\\\" + "nodeDict = {'nodeType': 'Point', \\\\'referenceCoordinates': [1.0, 0.0, 0.0], \\\\'initialCoordinates': [0.0, 2.0, 0.0], \\\\'name': 'example node'} \\\\mbs.AddNode(nodeDict)"
#                                isLambdaFunction = True
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetNodeNumber', cName='PyGetNodeNumber', 
                                description="get node's number by name (string)",
                                argList=['nodeName'],
                                example = "n = mbs.GetNodeNumber('example node')"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetNode', cName='PyGetNode', 
                                description="get node's dictionary by node number (type NodeIndex)",
                                argList=['nodeNumber'],
                                example = "nodeDict = mbs.GetNode(0)"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='ModifyNode', cName='PyModifyNode', 
                                description="modify node's dictionary by node number (type NodeIndex)",
                                argList=['nodeNumber','nodeDict'],
                                example = "mbs.ModifyNode(nodeNumber, nodeDict)"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetNodeDefaults', cName='PyGetNodeDefaults', 
                                description="get node's default values for a certain nodeType as (dictionary)",
                                argList=['typeName'],
                                example = "nodeType = 'Point'\\\\nodeDict = mbs.GetNodeDefaults(nodeType)"
                                )

#plr.DefPyFunctionAccess(cClass=classStr, pyName='CallNodeFunction', cName='PyCallNodeFunction', 
#                                description="call specific node function",
#                                argList=['nodeNumber', 'functionName', 'args'],
#                                defaultArgs=['', '', 'py::dict()']
#                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetNodeOutput', cName='PyGetNodeOutputVariable', 
                                description="get the ouput of the node specified with the OutputVariableType; output may be scalar or array (e.g. displacement vector)",
                                argList=['nodeNumber','variableType','configuration'],
                                defaultArgs=['','','ConfigurationType::Current'],
                                example = "mbs.GetNodeOutput(nodeNumber=0, variableType=exu.OutputVariableType.Displacement)"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetNodeODE2Index', cName='PyGetNodeODE2Index', 
                                description="get index in the global ODE2 coordinate vector for the first node coordinate of the specified node",
                                argList=['nodeNumber'],
                                example = "mbs.GetNodeODE2Index(nodeNumber=0)"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetNodeODE1Index', cName='PyGetNodeODE1Index', 
                                description="get index in the global ODE1 coordinate vector for the first node coordinate of the specified node",
                                argList=['nodeNumber'],
                                example = "mbs.GetNodeODE1Index(nodeNumber=0)"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetNodeAEIndex', cName='PyGetNodeAEIndex', 
                                description="get index in the global AE coordinate vector for the first node coordinate of the specified node",
                                argList=['nodeNumber'],
                                example = "mbs.GetNodeAEIndex(nodeNumber=0)"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetNodeParameter', cName='PyGetNodeParameter', 
                                description="get nodes's parameter from node number (type NodeIndex) and parameterName; parameter names can be found for the specific items in the reference manual; for visualization parameters, use a 'V' as a prefix",
                                argList=['nodeNumber', 'parameterName'],
                                example = "mbs.GetNodeParameter(0, 'referenceCoordinates')",
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetNodeParameter', cName='PySetNodeParameter', 
                                description="set parameter 'parameterName' of node with node number (type NodeIndex) to value; parameter names can be found for the specific items in the reference manual; for visualization parameters, use a 'V' as a prefix",
                                argList=['nodeNumber', 'parameterName', 'value'],
                                example = "mbs.SetNodeParameter(0, 'Vshow', True)",
                                )

plr.DefLatexFinishTable()
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#OBJECT
plr.sPy += "\n//        OBJECTS:\n"
plr.DefLatexStartClass(classStr+': Object', '', subSection=True,labelName='sec:mainsystem:object')

plr.AddDocu('This section provides functions for adding, reading and modifying objects, which can be bodies (mass point, '+
            'rigid body, finite element, ...), connectors (spring-damper or joint) or general objects. Objects provided '+
            'terms to the residual of equations resulting from every coordinate given by the nodes. Single-noded objects '+
            '(e.g.~mass point) provides exactly residual terms for its nodal coordinates. Connectors constrain or penalize '+
            'two markers, which can be, e.g., position, rigid or coordinate markers. Thus, the dependence of objects is '+
            'either on the coordinates of the marker-objects/nodes or on nodes which the objects possess themselves.')

plr.AddDocuCodeBlock(code="""
import exudyn as exu               #EXUDYN package including C++ core part
from exudyn.itemInterface import * #conversion of data to exudyn dictionaries
SC = exu.SystemContainer()         #container of systems
mbs = SC.AddSystem()               #add a new system to work with
nMP = mbs.AddNode(NodePoint2D(referenceCoordinates=[0,0]))
mbs.AddObject(ObjectMassPoint2D(physicsMass=10, nodeNumber=nMP ))
""")

plr.DefLatexStartTable(classStr+':objects')
#plr.DefPyFunctionAccess(cClass=classStr, pyName='AddObject', cName='[](MainSystem& mainSystem, py::object pyObject) {return mainSystem.AddMainObjectPyClass(pyObject); }', 
plr.DefPyFunctionAccess(cClass=classStr, pyName='AddObject', cName='AddMainObjectPyClass', 
                                description="add an object with objectDefinition from Python object class; returns (global) object number (type ObjectIndex) of newly added object",
                                argList=['pyObject'],
                                example = "item = MassPoint(name='heavy object', nodeNumber=0, physicsMass=100) \\\\mbs.AddObject(item) \\\\" + "objectDict = {'objectType': 'MassPoint', \\\\'physicsMass': 10, \\\\'nodeNumber': 0, \\\\'name': 'example object'} \\\\mbs.AddObject(objectDict)"
#                                isLambdaFunction = True
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetObjectNumber', cName='PyGetObjectNumber', 
                                description="get object's number by name (string)",
                                argList=['objectName'],
                                example = "n = mbs.GetObjectNumber('heavy object')"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetObject', cName='PyGetObject', 
                                description="get object's dictionary by object number (type ObjectIndex); NOTE: visualization parameters have a prefix 'V'; in order to also get graphicsData written, use addGraphicsData=True (which is by default False, as it would spoil the information)",
                                argList=['objectNumber','addGraphicsData'],
                                defaultArgs=['','False'],
                                example = "objectDict = mbs.GetObject(0)"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='ModifyObject', cName='PyModifyObject', 
                                description="modify object's dictionary by object number (type ObjectIndex); NOTE: visualization parameters have a prefix 'V'",
                                argList=['objectNumber','objectDict'],
                                example = "mbs.ModifyObject(objectNumber, objectDict)"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetObjectDefaults', cName='PyGetObjectDefaults', 
                                description="get object's default values for a certain objectType as (dictionary)",
                                argList=['typeName'],
                                example = "objectType = 'MassPoint'\\\\objectDict = mbs.GetObjectDefaults(objectType)"
                                )

#plr.DefPyFunctionAccess(cClass=classStr, pyName='CallObjectFunction', cName='PyCallObjectFunction', 
#                                description="call specific object function",
#                                argList=['objectNumber', 'functionName', 'args'],
#                                defaultArgs=['', '', 'py::dict()']
#                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetObjectOutput', cName='PyGetObjectOutputVariable', 
                                description="get object's current output variable from object number (type ObjectIndex) and OutputVariableType; for connectors, it can only be computed for exu.ConfigurationType.Current configuration!",
                                argList=['objectNumber', 'variableType', 'configuration'],
                                defaultArgs=['','','ConfigurationType::Current'],
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetObjectOutputBody', cName='PyGetObjectOutputVariableBody', 
                                description="get body's output variable from object number (type ObjectIndex) and OutputVariableType, using the localPosition as defined in the body, and as used in MarkerBody and SensorBody",
                                argList=['objectNumber', 'variableType', 'localPosition', 'configuration'],
                                defaultArgs=['','','(std::vector<Real>)Vector3D({0,0,0})','ConfigurationType::Current'],
                                example = "u = mbs.GetObjectOutputBody(objectNumber = 1, variableType = exu.OutputVariableType.Position, localPosition=[1,0,0], configuration = exu.ConfigurationType.Initial)"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetObjectOutputSuperElement', cName='PyGetObjectOutputVariableSuperElement', 
                                description="get output variable from mesh node number of object with type SuperElement (GenericODE2, FFRF, FFRFreduced - CMS) with specific OutputVariableType; the meshNodeNumber is the object's local node number, not the global node number!",
                                argList=['objectNumber', 'variableType', 'meshNodeNumber', 'configuration'],
                                defaultArgs=['','','','ConfigurationType::Current'],
                                example = "u = mbs.GetObjectOutputSuperElement(objectNumber = 1, variableType = exu.OutputVariableType.Position, meshNodeNumber = 12, configuration = exu.ConfigurationType.Initial)"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetObjectParameter', cName='PyGetObjectParameter', 
                                description="get objects's parameter from object number (type ObjectIndex) and parameterName; parameter names can be found for the specific items in the reference manual; for visualization parameters, use a 'V' as a prefix; NOTE that BodyGraphicsData cannot be get or set, use dictionary access instead",
                                argList=['objectNumber', 'parameterName'],
                                example = "mbs.GetObjectParameter(objectNumber = 0, parameterName = 'nodeNumber')",
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetObjectParameter', cName='PySetObjectParameter', 
                                description="set parameter 'parameterName' of object with object number (type ObjectIndex) to value;; parameter names can be found for the specific items in the reference manual; for visualization parameters, use a 'V' as a prefix; NOTE that BodyGraphicsData cannot be get or set, use dictionary access instead",
                                argList=['objectNumber', 'parameterName', 'value'],
                                example = "mbs.SetObjectParameter(objectNumber = 0, parameterName = 'Vshow', value=True)",
                                )

plr.DefLatexFinishTable()

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#MARKER
plr.sPy += "\n//        MARKER:\n"
plr.DefLatexStartClass(classStr+': Marker', '', subSection=True, labelName='sec:mainsystem:marker')

plr.AddDocu('This section provides functions for adding, reading and modifying markers. Markers define how to measure '+
            'primal kinematical quantities on objects or nodes (e.g., position, orientation or coordinates themselves), '+
            'and how to act on the quantities which are dual to the kinematical quantities (e.g., force, torque and '+
            'generalized forces). Markers provide unique interfaces for loads, sensors and constraints in order to address '+
            'these quantities independently of the structure of the object or node (e.g., rigid or flexible body).')

plr.AddDocuCodeBlock(code="""
import exudyn as exu               #EXUDYN package including C++ core part
from exudyn.itemInterface import * #conversion of data to exudyn dictionaries
SC = exu.SystemContainer()         #container of systems
mbs = SC.AddSystem()               #add a new system to work with
nMP = mbs.AddNode(NodePoint2D(referenceCoordinates=[0,0]))
mbs.AddObject(ObjectMassPoint2D(physicsMass=10, nodeNumber=nMP ))
mMP = mbs.AddMarker(MarkerNodePosition(nodeNumber = nMP))
""")

plr.DefLatexStartTable(classStr+':markers')

plr.DefPyFunctionAccess(cClass=classStr, pyName='AddMarker', cName='AddMainMarkerPyClass', 
                                description="add a marker with markerDefinition from Python marker class; returns (global) marker number (type MarkerIndex) of newly added marker",
                                argList=['pyObject'],
                                example = "item = MarkerNodePosition(name='my marker',nodeNumber=1) \\\\mbs.AddMarker(item)\\\\" + "markerDict = {'markerType': 'NodePosition', \\\\  'nodeNumber': 0, \\\\  'name': 'position0'}\\\\mbs.AddMarker(markerDict)"
#                                isLambdaFunction = True
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetMarkerNumber', cName='PyGetMarkerNumber', 
                                description="get marker's number by name (string)",
                                argList=['markerName'],
                                example = "n = mbs.GetMarkerNumber('my marker')"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetMarker', cName='PyGetMarker', 
                                description="get marker's dictionary by index",
                                argList=['markerNumber'],
                                example = "markerDict = mbs.GetMarker(0)"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='ModifyMarker', cName='PyModifyMarker', 
                                description="modify marker's dictionary by index",
                                argList=['markerNumber','markerDict'],
                                example = "mbs.ModifyMarker(markerNumber, markerDict)"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetMarkerDefaults', cName='PyGetMarkerDefaults', 
                                description="get marker's default values for a certain markerType as (dictionary)",
                                argList=['typeName'],
                                example = "markerType = 'NodePosition'\\\\markerDict = mbs.GetMarkerDefaults(markerType)"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetMarkerParameter', cName='PyGetMarkerParameter', 
                                description="get markers's parameter from markerNumber and parameterName; parameter names can be found for the specific items in the reference manual",
                                argList=['markerNumber', 'parameterName']
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetMarkerParameter', cName='PySetMarkerParameter', 
                                description="set parameter 'parameterName' of marker with markerNumber to value; parameter names can be found for the specific items in the reference manual",
                                argList=['markerNumber', 'parameterName', 'value']
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetMarkerOutput', cName='PyGetMarkerOutputVariable', 
                                description="get the ouput of the marker specified with the OutputVariableType; currently only provides Displacement, Position and Velocity for position based markers, and RotationMatrix, Rotation and AngularVelocity(Local) for markers providing orientation; Coordinates and Coordinates_t available for coordinate markers",
                                argList=['markerNumber','variableType','configuration'],
                                defaultArgs=['','','ConfigurationType::Current'],
                                example = "mbs.GetMarkerOutput(markerNumber=0, variableType=exu.OutputVariableType.Position)"
                                )


plr.DefLatexFinishTable()

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#LOAD
plr.sPy += "\n//        LOADS:\n"
plr.DefLatexStartClass(classStr+': Load', '', subSection=True, labelName='sec:mainsystem:load')

plr.AddDocu('This section provides functions for adding, reading and modifying operating loads. '+
            'Loads are used to act on the quantities which are dual to the primal kinematic quantities, '+
            'such as displacement and rotation. Loads represent, e.g., forces, torques or generalized forces.')

plr.AddDocuCodeBlock(code="""
import exudyn as exu               #EXUDYN package including C++ core part
from exudyn.itemInterface import * #conversion of data to exudyn dictionaries
SC = exu.SystemContainer()         #container of systems
mbs = SC.AddSystem()               #add a new system to work with
nMP = mbs.AddNode(NodePoint2D(referenceCoordinates=[0,0]))
mbs.AddObject(ObjectMassPoint2D(physicsMass=10, nodeNumber=nMP ))
mMP = mbs.AddMarker(MarkerNodePosition(nodeNumber = nMP))
mbs.AddLoad(Force(markerNumber = mMP, loadVector=[0.001,0,0]))
""")

plr.DefLatexStartTable(classStr+':loads')

plr.DefPyFunctionAccess(cClass=classStr, pyName='AddLoad', cName='AddMainLoadPyClass', 
                                description="add a load with loadDefinition from Python load class; returns (global) load number (type LoadIndex) of newly added load",
                                argList=['pyObject'],
                                example = "item = mbs.AddLoad(LoadForceVector(loadVector=[1,0,0], markerNumber=0, name='heavy load')) \\\\mbs.AddLoad(item)\\\\" + "loadDict = {'loadType': 'ForceVector',\\\\  'markerNumber': 0,\\\\  'loadVector': [1.0, 0.0, 0.0],\\\\  'name': 'heavy load'} \\\\mbs.AddLoad(loadDict)"
#                                isLambdaFunction = True
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetLoadNumber', cName='PyGetLoadNumber', 
                                description="get load's number by name (string)",
                                argList=['loadName'],
                                example = "n = mbs.GetLoadNumber('heavy load')"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetLoad', cName='PyGetLoad', 
                                description="get load's dictionary by index",
                                argList=['loadNumber'],
                                example = "loadDict = mbs.GetLoad(0)"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='ModifyLoad', cName='PyModifyLoad', 
                                description="modify load's dictionary by index",
                                argList=['loadNumber','loadDict'],
                                example = "mbs.ModifyLoad(loadNumber, loadDict)"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetLoadDefaults', cName='PyGetLoadDefaults', 
                                description="get load's default values for a certain loadType as (dictionary)",
                                argList=['typeName'],
                                example = "loadType = 'ForceVector'\\\\loadDict = mbs.GetLoadDefaults(loadType)"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetLoadValues', cName='PyGetLoadValues', 
                                description="Get current load values, specifically if user-defined loads are used; can be scalar or vector-valued return value",
                                argList=['loadNumber']
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetLoadParameter', cName='PyGetLoadParameter', 
                                description="get loads's parameter from loadNumber and parameterName; parameter names can be found for the specific items in the reference manual",
                                argList=['loadNumber', 'parameterName']
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetLoadParameter', cName='PySetLoadParameter', 
                                description="set parameter 'parameterName' of load with loadNumber to value; parameter names can be found for the specific items in the reference manual",
                                argList=['loadNumber', 'parameterName', 'value']
                                )

plr.DefLatexFinishTable()

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#SENSORS
plr.sPy += "\n//        SENSORS:\n"
plr.DefLatexStartClass(classStr+': Sensor', '', subSection=True, labelName='sec:mainsystem:sensor')

plr.AddDocu('This section provides functions for adding, reading and modifying operating sensors. '+
            'Sensors are used to measure information in nodes, objects, markers, and loads for output in a file.')

plr.AddDocuCodeBlock(code="""
import exudyn as exu               #EXUDYN package including C++ core part
from exudyn.itemInterface import * #conversion of data to exudyn dictionaries
SC = exu.SystemContainer()         #container of systems
mbs = SC.AddSystem()               #add a new system to work with
nMP = mbs.AddNode(NodePoint(referenceCoordinates=[0,0,0]))
mbs.AddObject(ObjectMassPoint(physicsMass=10, nodeNumber=nMP ))
mMP = mbs.AddMarker(MarkerNodePosition(nodeNumber = nMP))
mbs.AddLoad(Force(markerNumber = mMP, loadVector=[2,0,5]))
sMP = mbs.AddSensor(SensorNode(nodeNumber=nMP, storeInternal=True,
                               outputVariableType=exu.OutputVariableType.Position))
mbs.Assemble()
exu.SolveDynamic(mbs, exu.SimulationSettings())
from exudyn.plot import PlotSensor
PlotSensor(mbs, sMP, components=[0,1,2])
""")

plr.DefLatexStartTable(classStr+':sensors')

plr.DefPyFunctionAccess(cClass=classStr, pyName='AddSensor', cName='AddMainSensorPyClass',
                                description="add a sensor with sensor definition from Python sensor class; returns (global) sensor number (type SensorIndex) of newly added sensor",
                                argList=['pyObject'],
                                example = "item = mbs.AddSensor(SensorNode(sensorType= exu.SensorType.Node, nodeNumber=0, name='test sensor')) \\\\mbs.AddSensor(item)\\\\" + "sensorDict = {'sensorType': 'Node',\\\\  'nodeNumber': 0,\\\\  'fileName': 'sensor.txt',\\\\  'name': 'test sensor'} \\\\mbs.AddSensor(sensorDict)"
#                                isLambdaFunction = True
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetSensorNumber', cName='PyGetSensorNumber', 
                                description="get sensor's number by name (string)",
                                argList=['sensorName'],
                                example = "n = mbs.GetSensorNumber('test sensor')"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetSensor', cName='PyGetSensor', 
                                description="get sensor's dictionary by index",
                                argList=['sensorNumber'],
                                example = "sensorDict = mbs.GetSensor(0)"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='ModifySensor', cName='PyModifySensor', 
                                description="modify sensor's dictionary by index",
                                argList=['sensorNumber','sensorDict'],
                                example = "mbs.ModifySensor(sensorNumber, sensorDict)"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetSensorDefaults', cName='PyGetSensorDefaults', 
                                description="get sensor's default values for a certain sensorType as (dictionary)",
                                argList=['typeName'],
                                example = "sensorType = 'Node'\\\\sensorDict = mbs.GetSensorDefaults(sensorType)"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetSensorValues', cName='PyGetSensorValues', 
                                description="get sensors's values for configuration; can be a scalar or vector-valued return value!",
                                defaultArgs=['','ConfigurationType::Current'],
                                argList=['sensorNumber', 'configuration']
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetSensorStoredData', cName='PyGetSensorStoredData',
                                description="get sensors's internally stored data as matrix (all time points stored); rows are containing time and sensor values as obtained by sensor (e.g., time, and x, y, and z value of position)",
                                defaultArgs=[''],
                                argList=['sensorNumber']
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetSensorParameter', cName='PyGetSensorParameter', 
                                description="get sensors's parameter from sensorNumber and parameterName; parameter names can be found for the specific items in the reference manual",
                                argList=['sensorNumber', 'parameterName']
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetSensorParameter', cName='PySetSensorParameter', 
                                description="set parameter 'parameterName' of sensor with sensorNumber to value; parameter names can be found for the specific items in the reference manual",
                                argList=['sensorNumber', 'parameterName', 'value']
                                )

plr.DefLatexFinishTable() #Sensors

#now finalize pybind class, but do nothing on latex side (sL1 ignored)
plr2 = PyLatexRST()
plr2.DefPyFinishClass('MainSystem')
plr.sPy += plr2.PyStr() #only use Pybind string



#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
plr.CreateNewRSTfile('SystemData')
pyClassStr = 'SystemData'
classStr = 'Main'+pyClassStr
plr.DefPyStartClass(classStr,pyClassStr, '', labelName='sec:mbs:systemData')

plr.AddDocu('This is the data structure of a system which contains Objects (bodies/constraints/...), Nodes, Markers and Loads. '+
            'The SystemData structure allows advanced access to this data, which HAS TO BE USED WITH CARE, as unexpected results '+
            'and system crash might happen.')

plr.AddDocuCodeBlock(code="""
import exudyn as exu               #EXUDYN package including C++ core part
from exudyn.itemInterface import * #conversion of data to exudyn dictionaries
SC = exu.SystemContainer()         #container of systems
mbs = SC.AddSystem()               #add a new system to work with
nMP = mbs.AddNode(NodePoint(referenceCoordinates=[0,0,0]))
mbs.AddObject(ObjectMassPoint(physicsMass=10, nodeNumber=nMP ))
mMP = mbs.AddMarker(MarkerNodePosition(nodeNumber = nMP))
mbs.AddLoad(Force(markerNumber = mMP, loadVector=[2,0,5]))
mbs.Assemble()
exu.SolveDynamic(mbs, exu.SimulationSettings())

#obtain current ODE2 system vector (e.g. after static simulation finished):
u = mbs.systemData.GetODE2Coordinates()
#set initial ODE2 vector for next simulation:
mbs.systemData.SetODE2Coordinates(coordinates=u,
               configuration=exu.ConfigurationType.Initial)
#get detailed information as dictionary:
mbs.systemData.Info()
""")

plr.DefLatexStartTable(classStr)

plr.sPy += "\n//        General functions:\n"
#plr.sLatex += '\\\\ \n'+classStr+': General functions', 'These functions allow to obtain system information (e.g. for debug purposes)', subSection=True)

#+++++++++++++++++++++++++++++++++
#General functions:
plr.DefPyFunctionAccess(cClass=classStr, pyName='NumberOfLoads', cName='[](const MainSystemData& msd) {return msd.GetMainLoads().NumberOfItems(); }', 
                                description="return number of loads in system",
                                isLambdaFunction = True,
                                example = 'print(mbs.systemData.NumberOfLoads())')

plr.DefPyFunctionAccess(cClass=classStr, pyName='NumberOfMarkers', cName='[](const MainSystemData& msd) {return msd.GetMainMarkers().NumberOfItems(); }', 
                                description="return number of markers in system",
                                isLambdaFunction = True,
                                example = 'print(mbs.systemData.NumberOfMarkers())')

plr.DefPyFunctionAccess(cClass=classStr, pyName='NumberOfNodes', cName='[](const MainSystemData& msd) {return msd.GetMainNodes().NumberOfItems(); }', 
                                description="return number of nodes in system",
                                isLambdaFunction = True,
                                example = 'print(mbs.systemData.NumberOfNodes())')

plr.DefPyFunctionAccess(cClass=classStr, pyName='NumberOfObjects', cName='[](const MainSystemData& msd) {return msd.GetMainObjects().NumberOfItems(); }', 
                                description="return number of objects in system",
                                isLambdaFunction = True,
                                example = 'print(mbs.systemData.NumberOfObjects())')

plr.DefPyFunctionAccess(cClass=classStr, pyName='NumberOfSensors', cName='[](const MainSystemData& msd) {return msd.GetMainSensors().NumberOfItems(); }', 
                                description="return number of sensors in system",
                                isLambdaFunction = True,
                                example = 'print(mbs.systemData.NumberOfSensors())')

plr.DefPyFunctionAccess(cClass=classStr, pyName='ODE2Size', cName='PyODE2Size', 
                                description="get size of ODE2 coordinate vector for given configuration (only works correctly after mbs.Assemble() )",
                                argList=['configurationType'],
                                defaultArgs=['exu.ConfigurationType::Current'], #exu will be removed for binding
                                example = "print('ODE2 size=',mbs.systemData.ODE2Size())"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='ODE1Size', cName='PyODE1Size', 
                                description="get size of ODE1 coordinate vector for given configuration (only works correctly after mbs.Assemble() )",
                                argList=['configurationType'],
                                defaultArgs=['exu.ConfigurationType::Current'], #exu will be removed for binding
                                example = "print('ODE1 size=',mbs.systemData.ODE1Size())"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='AEsize', cName='PyAEsize', 
                                description="get size of AE coordinate vector for given configuration (only works correctly after mbs.Assemble() )",
                                argList=['configurationType'],
                                defaultArgs=['exu.ConfigurationType::Current'], #exu will be removed for binding
                                example = "print('AE size=',mbs.systemData.AEsize())"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='DataSize', cName='PyDataSize', 
                                description="get size of Data coordinate vector for given configuration (only works correctly after mbs.Assemble() )",
                                argList=['configurationType'],
                                defaultArgs=['exu.ConfigurationType::Current'], #exu will be removed for binding
                                example = "print('Data size=',mbs.systemData.DataSize())"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SystemSize', cName='PySystemSize', 
                                description="get size of System coordinate vector for given configuration (only works correctly after mbs.Assemble() )",
                                argList=['configurationType'],
                                defaultArgs=['exu.ConfigurationType::Current'], #exu will be removed for binding
                                example = "print('System size=',mbs.systemData.SystemSize())"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetTime', cName='PyGetStateTime', 
                                description="get configuration dependent time.",
                                argList=['configurationType'],
                                defaultArgs=['exu.ConfigurationType::Current'], #exu will be removed for binding
                                example = "mbs.systemData.GetTime(exu.ConfigurationType.Initial)"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetTime', cName='PySetStateTime', 
                                description="set configuration dependent time; use this access with care, e.g. in user-defined solvers.",
                                argList=['newTime','configurationType'],
                                defaultArgs=['', 'exu.ConfigurationType::Current'], #exu will be removed for binding
                                example = "mbs.systemData.SetTime(10., exu.ConfigurationType.Initial)"
                                )


#removed2021-05-01
# plr.DefPyFunctionAccess(cClass=classStr, pyName='GetCurrentTime', cName='PyGetCurrentTime', 
#                                 description="DEPRECATED; get current (simulation) time; time is updated in time integration solvers and in static solver; use this function e.g. during simulation to define time-dependent loads",
#                                 example = "mbs.systemData.GetCurrentTime()"
#                                 )

# plr.DefPyFunctionAccess(cClass=classStr, pyName='SetVisualizationTime', cName='PySetVisualizationTime', 
#                                 description="DEPRECATED; set time for render window (visualization)",
#                                 example = "mbs.systemData.SetVisualizationTime(1.3)"
#                                 )


plr.DefPyFunctionAccess(cClass=classStr, pyName='Info', cName='[](const MainSystemData& msd) {pout << msd.PyInfoDetailed(); }', 
                                description="print detailed system information for every item; for short information use print(mbs)",
                                isLambdaFunction = True,
                                example = 'mbs.systemData.Info()')



plr.DefLatexFinishTable()

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
plr.sPy += "\n//        Coordinate access:\n"
#start new Latex/RST subsection:
plr.DefLatexStartClass(pyClassStr+': Access coordinates', '', subSection=True, labelName='sec:mbs:systemData:coordinates')

plr.AddDocu('This section provides access functions to global coordinate vectors. Assigning invalid values or using '+
            'wrong vector size might lead to system crash and unexpected results.')

plr.DefLatexStartTable(classStr+':coordinate access')
#+++++++++++++++++++++++++++++++++
#coordinate access functions:

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetODE2Coordinates', cName='GetODE2Coords', 
                                description="get ODE2 system coordinates (displacements) for given configuration (default: exu.Configuration.Current)",
                                argList=['configuration'],
                                defaultArgs=['exu.ConfigurationType::Current'],
                                example = "uCurrent = mbs.systemData.GetODE2Coordinates()"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetODE2Coordinates', cName='SetODE2Coords', 
                                description="set ODE2 system coordinates (displacements) for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!",
                                argList=['coordinates','configuration'],
                                defaultArgs=['','exu.ConfigurationType::Current'],
                                example = "mbs.systemData.SetODE2Coordinates(uCurrent)"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetODE2Coordinates_t', cName='GetODE2Coords_t', 
                                description="get ODE2 system coordinates (velocities) for given configuration (default: exu.Configuration.Current)",
                                argList=['configuration'],
                                defaultArgs=['exu.ConfigurationType::Current'],
                                example = "vCurrent = mbs.systemData.GetODE2Coordinates_t()"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetODE2Coordinates_t', cName='SetODE2Coords_t', 
                                description="set ODE2 system coordinates (velocities) for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!",
                                argList=['coordinates','configuration'],
                                defaultArgs=['','exu.ConfigurationType::Current'],
                                example = "mbs.systemData.SetODE2Coordinates_t(vCurrent)"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetODE2Coordinates_tt', cName='GetODE2Coords_tt', 
                                description="get ODE2 system coordinates (accelerations) for given configuration (default: exu.Configuration.Current)",
                                argList=['configuration'],
                                defaultArgs=['exu.ConfigurationType::Current'],
                                example = "vCurrent = mbs.systemData.GetODE2Coordinates_tt()"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetODE2Coordinates_tt', cName='SetODE2Coords_tt', 
                                description="set ODE2 system coordinates (accelerations) for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!",
                                argList=['coordinates','configuration'],
                                defaultArgs=['','exu.ConfigurationType::Current'],
                                example = "mbs.systemData.SetODE2Coordinates_tt(aCurrent)"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetODE1Coordinates', cName='GetODE1Coords', 
                                description="get ODE1 system coordinates (displacements) for given configuration (default: exu.Configuration.Current)",
                                argList=['configuration'],
                                defaultArgs=['exu.ConfigurationType::Current'],
                                example = "qCurrent = mbs.systemData.GetODE1Coordinates()"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetODE1Coordinates', cName='SetODE1Coords', 
                                description="set ODE1 system coordinates (velocities) for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!",
                                argList=['coordinates','configuration'],
                                defaultArgs=['','exu.ConfigurationType::Current'],
                                example = "mbs.systemData.SetODE1Coordinates_t(qCurrent)"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetODE1Coordinates_t', cName='GetODE1Coords_t', 
                                description="get ODE1 system coordinates (velocities) for given configuration (default: exu.Configuration.Current)",
                                argList=['configuration'],
                                defaultArgs=['exu.ConfigurationType::Current'],
                                example = "qCurrent = mbs.systemData.GetODE1Coordinates_t()"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetODE1Coordinates_t', cName='SetODE1Coords_t', 
                                description="set ODE1 system coordinates (displacements) for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!",
                                argList=['coordinates','configuration'],
                                defaultArgs=['','exu.ConfigurationType::Current'],
                                example = "mbs.systemData.SetODE1Coordinates(qCurrent)"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetAECoordinates', cName='GetAECoords', 
                                description="get algebraic equations (AE) system coordinates for given configuration (default: exu.Configuration.Current)",
                                argList=['configuration'],
                                defaultArgs=['exu.ConfigurationType::Current'],
                                example = "lambdaCurrent = mbs.systemData.GetAECoordinates()"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetAECoordinates', cName='SetAECoords', 
                                description="set algebraic equations (AE) system coordinates for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!",
                                argList=['coordinates','configuration'],
                                defaultArgs=['','exu.ConfigurationType::Current'],
                                example = "mbs.systemData.SetAECoordinates(lambdaCurrent)"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetDataCoordinates', cName='GetDataCoords', 
                                description="get system data coordinates for given configuration (default: exu.Configuration.Current)",
                                argList=['configuration'],
                                defaultArgs=['exu.ConfigurationType::Current'],
                                example = "dataCurrent = mbs.systemData.GetDataCoordinates()"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetDataCoordinates', cName='SetDataCoords', 
                                description="set system data coordinates for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!",
                                argList=['coordinates','configuration'],
                                defaultArgs=['','exu.ConfigurationType::Current'],
                                example = "mbs.systemData.SetDataCoordinates(dataCurrent)"
                                )



plr.DefPyFunctionAccess(cClass=classStr, pyName='GetSystemState', cName='PyGetSystemState', 
                                description="get system state for given configuration (default: exu.Configuration.Current); state vectors do not include the non-state derivatives ODE1_t and ODE2_tt and the time; function is copying data - not highly efficient; format of pyList: [ODE2Coords, ODE2Coords_t, ODE1Coords, AEcoords, dataCoords]",
                                argList=['configuration'],
                                defaultArgs=['exu.ConfigurationType::Current'], #exu will be removed for binding
                                example = "sysStateList = mbs.systemData.GetSystemState()"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetSystemState', cName='PySetSystemState', 
                                description="set system data coordinates for given configuration (default: exu.Configuration.Current); invalid list of vectors / vector size may lead to system crash; write access to state vectors (but not the non-state derivatives ODE1_t and ODE2_tt and the time); function is copying data - not highly efficient; format of pyList: [ODE2Coords, ODE2Coords_t, ODE1Coords, AEcoords, dataCoords]",
                                argList=['systemStateList','configuration'],
                                defaultArgs=['','exu.ConfigurationType::Current'], #exu will be removed for binding
                                example = "mbs.systemData.SetSystemState(sysStateList, configuration = exu.ConfigurationType.Initial)"
                                )


plr.DefLatexFinishTable()

#+++++++++++++++++++++++++++++++++
#LTG-functions:
plr.sPy += "\n//        LTG readout functions:\n"
plr.DefLatexStartClass(pyClassStr+': Get object LTG coordinate mappings', '', subSection=True, labelName='sec:systemData:ObjectLTG')

plr.AddDocu('This section provides access functions the \\ac{LTG}-lists for every object (body, constraint, ...) '+
            'in the system. For details on the \\ac{LTG} mapping, see \\refSection{sec:overview:ltgmapping}')

plr.DefLatexStartTable(classStr+':object LTG coordinate mappings')

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetObjectLTGODE2', cName='PyGetObjectLocalToGlobalODE2', 
                                description="get local-to-global coordinate mapping (list of global coordinate indices) for ODE2 coordinates; only available after Assemble()",
                                argList=['objectNumber'],
                                example = "ltgObject4 = mbs.systemData.GetObjectLTGODE2(4)"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetObjectLTGODE1', cName='PyGetObjectLocalToGlobalODE1', 
                                description="get local-to-global coordinate mapping (list of global coordinate indices) for ODE1 coordinates; only available after Assemble()",
                                argList=['objectNumber'],
                                example = "ltgObject4 = mbs.systemData.GetObjectLTGODE1(4)"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetObjectLTGAE', cName='PyGetObjectLocalToGlobalAE', 
                                description="get local-to-global coordinate mapping (list of global coordinate indices) for algebraic equations (AE) coordinates; only available after Assemble()",
                                argList=['objectNumber'],
                                example = "ltgObject4 = mbs.systemData.GetObjectLTGODE2(4)"
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetObjectLTGData', cName='PyGetObjectLocalToGlobalData', 
                                description="get local-to-global coordinate mapping (list of global coordinate indices) for data coordinates; only available after Assemble()",
                                argList=['objectNumber'],
                                example = "ltgObject4 = mbs.systemData.GetObjectLTGData(4)"
                                )

plr.DefLatexFinishTable()

#now finalize pybind class, but do nothing on latex side (sL1 ignored)
plr2 = PyLatexRST()
plr2.DefPyFinishClass('SystemData')
plr.sPy += plr2.PyStr() #only use Pybind string


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
plr.CreateNewRSTfile('GeneralContact')
classStr = 'PyGeneralContact'
pyClassStr = 'GeneralContact'
plr.DefPyStartClass(classStr, pyClassStr, '',labelName='sec:GeneralContact')

plr.AddDocu('Structure to define general and highly efficient contact functionality in multibody systems'+
            '\\footnote{Note that GeneralContact is still developed, use with care.}. For further explanations '+
            'and theoretical backgrounds, see \\refSection{secContactTheory}.')

plr.AddDocuCodeBlock(code="""
#...
#code snippet, must be placed anywhere before mbs.Assemble()
#Add GeneralContact to mbs:
gContact = mbs.AddGeneralContact()
#Add contact elements, e.g.:
gContact.AddSphereWithMarker(...) #use appropriate arguments
gContact.SetFrictionPairings(...) #set friction pairings and adjust searchTree if needed.
""")

plr.DefLatexStartTable(pyClassStr)

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetPythonObject', cName='GetPythonObject', 
                                description="convert member variables of GeneralContact into dictionary; use this for debug only!")

plr.DefPyFunctionAccess(cClass=classStr, pyName='Reset', cName='Reset', 
                                argList=['freeMemory'],
                                defaultArgs=['True'],
                                description="remove all contact objects and reset contact parameters")

plr.sPy +=  '        .def_readwrite("isActive", &PyGeneralContact::isActive, py::return_value_policy::reference)\n' 
plr.DefLatexDataAccess('isActive','default = True (compute contact); if isActive=False, no contact computation is performed for this contact set ')

plr.sPy +=  '        .def_readwrite("verboseMode", &PyGeneralContact::verboseMode, py::return_value_policy::reference)\n' 
plr.DefLatexDataAccess('verboseMode','default = 0; verboseMode = 1 or higher outputs useful information on the contact creation and computation ')

plr.sPy +=  '        .def_readwrite("visualization", &PyGeneralContact::visualization, py::return_value_policy::reference)\n' 
plr.DefLatexDataAccess('visualization','access visualization data structure ')

plr.sPy +=  '        .def_property("resetSearchTreeInterval", &PyGeneralContact::GetResetSearchTreeInterval, &PyGeneralContact::SetResetSearchTreeInterval)\n' 
plr.DefLatexDataAccess('resetSearchTreeInterval','(default=10000) number of search tree updates (contact computation steps) after which the search tree cells are re-created; this costs some time, will free memory in cells that are not needed any more ')

#plr.sPy +=  '        .def_readwrite("intraSpheresContact", &PyGeneralContact::settings.intraSpheresContact, py::return_value_policy::reference)\n' 
plr.sPy +=  '        .def_property("sphereSphereContact", &PyGeneralContact::GetSphereSphereContact, &PyGeneralContact::SetSphereSphereContact)\n' 
plr.DefLatexDataAccess('sphereSphereContact','activate/deactivate contact between spheres ')

plr.sPy +=  '        .def_property("sphereSphereFrictionRecycle", &PyGeneralContact::GetSphereSphereFrictionRecycle, &PyGeneralContact::SetSphereSphereFrictionRecycle)\n' 
plr.DefLatexDataAccess('sphereSphereFrictionRecycle','False: compute static friction force based on tangential velocity; True: recycle friction from previous PostNewton step, which greatly improves convergence, but may lead to unphysical artifacts; will be solved in future by step reduction ')

plr.sPy +=  '        .def_property("minRelDistanceSpheresTriangles", &PyGeneralContact::GetMinRelDistanceSpheresTriangles, &PyGeneralContact::SetMinRelDistanceSpheresTriangles)\n' 
plr.DefLatexDataAccess('minRelDistanceSpheresTriangles','(default=1e-10) tolerance (relative to sphere radiues) below which the contact between triangles and spheres is ignored; used for spheres directly attached to triangles ')

plr.sPy +=  '        .def_property("frictionProportionalZone", &PyGeneralContact::GetFrictionProportionalZone, &PyGeneralContact::SetFrictionProportionalZone)\n' 
plr.DefLatexDataAccess('frictionProportionalZone','(default=0.001) velocity $v_{\mu,reg}$ upon which the dry friction coefficient is interpolated linearly (regularized friction model); must be greater 0; very small values cause oscillations in friction force ')

plr.sPy +=  '        .def_property("frictionVelocityPenalty", &PyGeneralContact::GetFrictionVelocityPenalty, &PyGeneralContact::SetFrictionVelocityPenalty)\n' 
plr.DefLatexDataAccess('frictionVelocityPenalty','(default=1e3) regularization factor for friction [N/(m$^2 \cdot$m/s) ];$k_{\mu,reg}$, multiplied with tangential velocity to compute friciton force as long as it is smaller than $\mu$ times contact force; large values cause oscillations in friction force ')

plr.sPy +=  '        .def_property("excludeOverlappingTrigSphereContacts", &PyGeneralContact::GetExcludeOverlappingTrigSphereContacts, &PyGeneralContact::SetExcludeOverlappingTrigSphereContacts)\n' 
plr.DefLatexDataAccess('excludeOverlappingTrigSphereContacts','(default=True) for consistent, closed meshes, we can exclude overlapping contact triangles (which would cause holes if mesh is overlapping and not consistent!!!) ')

plr.sPy +=  '        .def_property("excludeDuplicatedTrigSphereContactPoints", &PyGeneralContact::GetExcludeDuplicatedTrigSphereContactPoints, &PyGeneralContact::SetExcludeDuplicatedTrigSphereContactPoints)\n' 
plr.DefLatexDataAccess('excludeDuplicatedTrigSphereContactPoints','(default=False) run additional checks for double contacts at edges or vertices, being more accurate but can cause additional costs if many contacts ')

plr.sPy +=  '        .def_property("ancfCableUseExactMethod", &PyGeneralContact::GetAncfCableUseExactMethod, &PyGeneralContact::SetAncfCableUseExactMethod)\n' 
plr.DefLatexDataAccess('ancfCableUseExactMethod','(default=True) if True, uses exact computation of intersection of 3rd order polynomials and contacting circles ')

plr.sPy +=  '        .def_property("ancfCableNumberOfContactSegments", &PyGeneralContact::GetAncfCableNumberOfContactSegments, &PyGeneralContact::SetAncfCableNumberOfContactSegments)\n' 
plr.DefLatexDataAccess('ancfCableNumberOfContactSegments','(default=1) number of segments to be used in case that ancfCableUseExactMethod=False; maximum number of segments=3 ')

plr.sPy +=  '        .def_property("ancfCableMeasuringSegments", &PyGeneralContact::GetAncfCableMeasuringSegments, &PyGeneralContact::SetAncfCableMeasuringSegments)\n' 
plr.DefLatexDataAccess('ancfCableMeasuringSegments','(default=20) number of segments used to approximate geometry for ANCFCable2D elements for measuring with ShortestDistanceAlongLine; with 20 segments the relative error due to approximation as compared to 10 segments usually stays below 1e-8 ')


# plr.DefPyFunctionAccess(cClass=classStr, pyName='FinalizeContact', cName='PyFinalizeContact', 
#                                 argList=['mainSystem','searchTreeSize','frictionPairingsInit','searchTreeBoxMin','searchTreeBoxMax'],
#                                 defaultArgs=['','','', '(std::vector<Real>)Vector3D( EXUstd::MAXREAL )','(std::vector<Real>)Vector3D( EXUstd::LOWESTREAL )'],
#                                 description="WILL CHANGE IN FUTURE: Call this function after mbs.Assemble(); precompute some contact arrays (mainSystem needed) and set up necessary parameters for contact: friction, SearchTree, etc.; done after all contacts have been added; function performs checks; empty box will autocompute size!")
plr.DefPyFunctionAccess(cClass=classStr, pyName='SetFrictionPairings', cName='SetFrictionPairings', 
                               argList=['frictionPairings'],
                               example='\\#set 3 surface friction types, all being 0.1:\\\\gContact.SetFrictionPairings(0.1*np.ones((3,3)));',
                               description="set Coulomb friction coefficients for pairings of materials (e.g., use material 0,1, then the entries (0,1) and (1,0) define the friction coefficients for this pairing); matrix should be symmetric!")
                
plr.DefPyFunctionAccess(cClass=classStr, pyName='SetFrictionProportionalZone', cName='SetFrictionProportionalZone', 
                               argList=['frictionProportionalZone'],
                               description="regularization for friction (m/s); used for all contacts")
                                      
plr.DefPyFunctionAccess(cClass=classStr, pyName='SetSearchTreeCellSize', cName='SetSearchTreeCellSize', 
                               argList=['numberOfCells'],
                               example='gContact.SetSearchTreeInitSize([10,10,10])',
                               description="set number of cells of search tree (boxed search) in x, y and z direction")
                                                      
plr.DefPyFunctionAccess(cClass=classStr, pyName='SetSearchTreeBox', cName='SetSearchTreeBox', 
                               argList=['pMin','pMax'],
                               example='gContact.SetSearchTreeBox(pMin=[-1,-1,-1],\\\\ \\TAB pMax=[1,1,1])',
                               description="set geometric dimensions of searchTreeBox (point with minimum coordinates and point with maximum coordinates); if this box becomes smaller than the effective contact objects, contact computations may slow down significantly")
                                                      
plr.DefPyFunctionAccess(cClass=classStr, pyName='AddSphereWithMarker', cName='AddSphereWithMarker', 
                                argList=['markerIndex','radius','contactStiffness','contactDamping','frictionMaterialIndex'],
                                description="add contact object using a marker (Position or Rigid), radius and contact/friction parameters and return localIndex of the contact item in GeneralContact; frictionMaterialIndex refers to frictionPairings in GeneralContact; contact is possible between spheres (circles in 2D) (if intraSphereContact = True), spheres and triangles and between sphere (=circle) and ANCFCable2D; contactStiffness is computed as serial spring between contacting objects, while damping is computed as a parallel damper")
                                                      
plr.DefPyFunctionAccess(cClass=classStr, pyName='AddANCFCable', cName='AddANCFCable', 
                                argList=['objectIndex','halfHeight','contactStiffness','contactDamping','frictionMaterialIndex'],
                                description="add contact object for an ANCF cable element, using the objectIndex of the cable element and the cable's half height as an additional distance to contacting objects (currently not causing additional torque in case of friction), and return localIndex of the contact item in GeneralContact; currently only contact with spheres (circles in 2D) possible; contact computed using exact geometry of elements, finding max 3 intersecting contact regions")

plr.DefPyFunctionAccess(cClass=classStr, pyName='AddTrianglesRigidBodyBased', cName='PyAddTrianglesRigidBodyBased', 
                                argList=['rigidBodyMarkerIndex','contactStiffness','contactDamping','frictionMaterialIndex','pointList','triangleList'],
                                description="add contact object using a rigidBodyMarker (of a body), contact/friction parameters, a list of points (as 3D numpy arrays or lists; coordinates relative to rigidBodyMarker) and a list of triangles (3 indices as numpy array or list) according to a mesh attached to the rigidBodyMarker; returns starting local index of trigsRigidBodyBased at which the triangles are stored; mesh can be produced with GraphicsData2TrigsAndPoints(...); contact is possible between sphere (circle) and Triangle but yet not between triangle and triangle; frictionMaterialIndex refers to frictionPairings in GeneralContact; contactStiffness is computed as serial spring between contacting objects, while damping is computed as a parallel damper (otherwise the smaller damper would always dominate); the triangle normal must point outwards, with the normal of a triangle given with local points (p0,p1,p2) defined as n=(p1-p0) x (p2-p0), see function ComputeTriangleNormal(...)")

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#access functions:
plr.DefPyFunctionAccess(cClass=classStr, pyName='GetItemsInBox', cName='PyGetItemsInBox', 
                                argList=['pMin','pMax'],
                                example='gContact.GetItemsInBox(pMin=[0,1,1],\\\\ \\TAB pMax=[2,3,2])',
                                description="Get all items in box defined by minimum coordinates given in pMin and maximum coordinates given by pMax, accepting 3D lists or numpy arrays; in case that no objects are found, False is returned; otherwise, a dictionary is returned, containing numpy arrays with indices of obtained MarkerBasedSpheres, TrigsRigidBodyBased, ANCFCable2D, ...; the indices refer to the local index in GeneralContact which can be evaluated e.g. by GetMarkerBasedSphere(localIndex)")

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetMarkerBasedSphere', cName='PyGetMarkerBasedSphere', 
                                argList=['localIndex'],
                                description="Get dictionary with position, radius and markerIndex for markerBasedSphere index, as returned e.g. from GetItemsInBox")

plr.DefPyFunctionAccess(cClass=classStr, pyName='ShortestDistanceAlongLine', cName='PyShortestDistanceAlongLine', 
                                argList=['pStart','direction','minDistance','maxDistance','asDictionary','cylinderRadius','typeIndex'],
                                defaultArgs=['(std::vector<Real>)Vector3D({0,0,0})','(std::vector<Real>)Vector3D({1,0,0})','-1e-7','1e7','False','0','Contact::IndexEndOfEnumList'],
                                description="Find shortest distance to contact objects in GeneralContact along line with pStart (given as 3D list or numpy array) and direction (as 3D list or numpy array with no need to be normalized); the function returns the distance which is >= minDistance and < maxDistance; in case of beam elements, it measures the distance to the beam centerline; the distance is measured from pStart along given direction and can also be negative; if no item is found along line, the maxDistance is returned; if asDictionary=False, the result is a float, while otherwise details are returned as dictionary (including distance, velocityAlongLine (which is the object velocity in given direction and may be different from the time derivative of the distance; works similar to a laser Doppler vibrometer - LDV), itemIndex and itemType in GeneralContact); the cylinderRadius, if not equal to 0, will be used for spheres to find closest sphere along cylinder with given point and direction; the typeIndex can be set to a specific contact type, e.g., which are searched for (otherwise all objects are considered)")


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                                                      
plr.DefPyFunctionAccess(cClass=classStr, pyName='__repr__', cName='[](const PyGeneralContact &item) {\n            return EXUstd::ToString(item); }', 
                                description="return the string representation of the GeneralContact, containing basic information and statistics",
                                isLambdaFunction = True)


#++++++++++++++++
plr.DefPyFinishClass('GeneralContact')


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#documentation and pybindings for VisuGeneralContact
classStr = 'VisuGeneralContact'
pyClassStr = 'VisuGeneralContact'
plr.DefPyStartClass(classStr, pyClassStr, 'This structure may contains some visualization parameters in future. '+
                    'Currently, all visualization settings are controlled via SC.visualizationSettings', 
                    subSection=True, labelName='sec:GeneralContact:visualization')

plr.DefLatexStartTable(pyClassStr)

plr.DefPyFunctionAccess(cClass=classStr, pyName='Reset', cName='Reset', 
                                description="reset visualization parameters to default values")

# plr.sPy +=  '        .def_readwrite("spheresMarkerBasedDraw", &VisuGeneralContact::spheresMarkerBasedDraw, py::return_value_policy::reference)\n' 
# plr.DefLatexDataAccess('spheresMarkerBasedDraw','default = False; if True, markerBased spheres are drawn with given resolution and color ')

# plr.sPy +=  '        .def_readwrite("spheresMarkerBasedResolution", &VisuGeneralContact::spheresMarkerBasedResolution, py::return_value_policy::reference)\n' 
# plr.DefLatexDataAccess('spheresMarkerBasedResolution','default = 4; integer value for number of triangles per circumference of markerBased spheres; higher values leading to smoother spheres but higher graphics costs ')

# plr.sPy +=  '        .def_readwrite("spheresMarkerBasedColor", &VisuGeneralContact::spheresMarkerBasedColor, py::return_value_policy::reference)\n' 
# plr.DefLatexDataAccess('spheresMarkerBasedColor','vector with 4 floats (Float4) for color of markerBased spheres ')

#++++++++++++++++
plr.DefPyFinishClass('GeneralContact')




plr.CreateNewRSTfile('DataStructures')

plr.AddDocu(text="""
This section describes a set of special data structures which are used in the Python-C++ interface, 
such as a MatrixContainer for dense/sparse matrices or a list of 3D vectors. 
Note that there are many native data types, such as lists, dicts and numpy arrays (e.g. 3D vectors), 
which are not described here as they are native to Pybind11, but can be passed as arguments when appropriate.
""", section='Data structures', sectionLevel=1,sectionLabel='sec:cinterface:dataStructures')


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#documentation and pybindings for MatrixContainer
classStr = 'PyMatrixContainer'
pyClassStr = 'MatrixContainer'
plr.DefPyStartClass(classStr, pyClassStr, 'The MatrixContainer is a versatile representation for dense and sparse matrices.',
                    subSection=True)

plr.AddDocuCodeBlock(code="""
#Create empty MatrixContainer:
mc = MatrixContainer()

#Create MatrixContainer with dense matrix:
#matrix can be a list of lists or a numpy array, e.g.:
matrix = np.eye(6)
mc = MatrixContainer(matrix)

#Set with dense pyArray (a numpy array): 
mc.SetWithDenseMatrix(pyArray, useDenseMatrix = True)
""")

plr.DefLatexStartTable(pyClassStr)

plr.sPy += '        .def(py::init<const py::object&>(), py::arg("matrix"))\n' #constructor with numpy array or list of lists

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetWithDenseMatrix', cName='SetWithDenseMatrix', 
                                argList=['pyArray','useDenseMatrix'],
                                defaultArgs=['','False'],
                                description="set MatrixContainer with dense numpy array; array (=matrix) contains values and matrix size information; if useDenseMatrix=True, matrix will be stored internally as dense matrix, otherwise it will be converted and stored as sparse matrix (which may speed up computations for larger problems)")
                                                      
plr.DefPyFunctionAccess(cClass=classStr, pyName='SetWithSparseMatrixCSR', cName='SetWithSparseMatrixCSR', 
                                argList=['numberOfRowsInit', 'numberOfColumnsInit', 'pyArrayCSR','useDenseMatrix'],
                                defaultArgs=['','','','True'],
                                description="set with sparse CSR matrix format: numpy array 'pyArrayCSR' contains sparse triplet (row, col, value) per row; numberOfRows and numberOfColumns given extra; if useDenseMatrix=True, matrix will be converted and stored internally as dense matrix, otherwise it will be stored as sparse matrix")
                                                      
plr.DefPyFunctionAccess(cClass=classStr, pyName='GetPythonObject', cName='GetPythonObject', 
                                description="convert MatrixContainer to numpy array (dense) or dictionary (sparse): containing nr. of rows, nr. of columns, numpy matrix with sparse triplets")

plr.DefPyFunctionAccess(cClass=classStr, pyName='Convert2DenseMatrix', cName='Convert2DenseMatrix', 
                                description="convert MatrixContainer to dense numpy array (SLOW and may fail for too large sparse matrices)")

plr.DefPyFunctionAccess(cClass=classStr, pyName='UseDenseMatrix', cName='UseDenseMatrix', 
                                description="returns True if dense matrix is used, otherwise False")

plr.DefPyFunctionAccess(cClass=classStr, pyName='__repr__', cName='[](const PyMatrixContainer &item) {\n            return EXUstd::ToString(item.GetPythonObject()); }', 
                                description="return the string representation of the MatrixContainer",
                                isLambdaFunction = True)

#++++++++++++++++
plr.DefPyFinishClass('MatrixContainer')


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#documentation and pybindings for PyVector3DList


classStr = 'PyVector3DList'
pyClassStr = 'Vector3DList'
plr.DefPyStartClass(classStr, pyClassStr, "The Vector3DList is used to represent lists of 3D vectors. This is used to transfer such lists from Python to C++." +
        ' \\\\ \\\\ Usage: \\bi\n'+
        '  \\item Create empty \\texttt{Vector3DList} with \\texttt{x = Vector3DList()} \n'+
        '  \\item Create \\texttt{Vector3DList} with list of numpy arrays:\\\\\\texttt{x = Vector3DList([ numpy.array([1.,2.,3.]), numpy.array([4.,5.,6.]) ])}\n'+
        '  \\item Create \\texttt{Vector3DList} with list of lists \\texttt{x = Vector3DList([[1.,2.,3.], [4.,5.,6.]])}\n'+
        '  \\item Append item: \\texttt{x.Append([0.,2.,4.])}\n'+
        '  \\item Convert into list of numpy arrays: \\texttt{x.GetPythonObject()}\n'+
        '\\ei\n', subSection=True)

plr.DefLatexStartTable(pyClassStr)

plr.sPy += '        .def(py::init<const py::object&>(), py::arg("listOfArrays"))\n' #constructor with numpy array or list of lists

plr.DefPyFunctionAccess(cClass=classStr, pyName='Append', cName='PyAppend', 
                               argList=['pyArray'],
                               description="add single array or list to Vector3DList; array or list must have appropriate dimension!")
                                                                                                            
plr.DefPyFunctionAccess(cClass=classStr, pyName='GetPythonObject', cName='GetPythonObject', 
                               description="convert Vector3DList into (copied) list of numpy arrays")

plr.DefPyFunctionAccess(cClass=classStr, pyName='__len__', 
                               cName='[](const PyVector3DList &item) {\n            return item.NumberOfItems(); }', 
                               description="return length of the Vector3DList, using len(data) where data is the Vector3DList",
                               isLambdaFunction = True)

plr.DefPyFunctionAccess(cClass=classStr, pyName='__setitem__', 
                               cName='[](PyVector3DList &item, Index index, const py::object& vector) {\n            item.PySetItem(index, vector); }', 
                               description="set list item 'index' with data, write: data[index] = ...",
                               isLambdaFunction = True)

plr.DefPyFunctionAccess(cClass=classStr, pyName='__getitem__', 
                               cName='[](const PyVector3DList &item, Index index) {\n            return py::array_t<Real>(item[index].NumberOfItems(), item[index].GetDataPointer()); }', 
                               description="get copy of list item with 'index' as vector",
                               isLambdaFunction = True)

plr.DefPyFunctionAccess(cClass=classStr, pyName='__repr__', 
                               cName='[](const PyVector3DList &item) {\n            return EXUstd::ToString(item.GetPythonObject()); }', 
                               description="return the string representation of the Vector3DList data, e.g.: print(data)",
                               isLambdaFunction = True)

#++++++++++++++++
plr.DefPyFinishClass('PyVector3DList')

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#documentation and pybindings for PyVector2DList
classStr = 'PyVector2DList'
pyClassStr = 'Vector2DList'
plr.DefPyStartClass(classStr, pyClassStr, "The Vector2DList is used to represent lists of 2D vectors. This is used to transfer such lists from Python to C++." +
        ' \\\\ \\\\ Usage: \\bi\n'+
        '  \\item Create empty \\texttt{Vector2DList} with \\texttt{x = Vector2DList()} \n'+
        '  \\item Create \\texttt{Vector2DList} with list of numpy arrays:\\\\\\texttt{x = Vector2DList([ numpy.array([1.,2.]), numpy.array([4.,5.]) ])}\n'+
        '  \\item Create \\texttt{Vector2DList} with list of lists \\texttt{x = Vector2DList([[1.,2.], [4.,5.]])}\n'+
        '  \\item Append item: \\texttt{x.Append([0.,2.])}\n'+
        '  \\item Convert into list of numpy arrays: \\texttt{x.GetPythonObject()}\n'+
        '  \\item similar to Vector3DList !\n'+
        '\\ei\n', subSection=True)

plr.DefLatexStartTable(pyClassStr)

plr.sPy += '        .def(py::init<const py::object&>(), py::arg("listOfArrays"))\n' #constructor with numpy array or list of lists

plr.DefPyFunctionAccess(cClass=classStr, pyName='Append', cName='PyAppend', 
                               argList=['pyArray'],
                               description="add single array or list to Vector2DList; array or list must have appropriate dimension!")
                                                                                                            
plr.DefPyFunctionAccess(cClass=classStr, pyName='GetPythonObject', cName='GetPythonObject', 
                               description="convert Vector2DList into (copied) list of numpy arrays")

plr.DefPyFunctionAccess(cClass=classStr, pyName='__len__', 
                               cName='[](const PyVector2DList &item) {\n            return item.NumberOfItems(); }', 
                               description="return length of the Vector2DList, using len(data) where data is the Vector2DList",
                               isLambdaFunction = True)

plr.DefPyFunctionAccess(cClass=classStr, pyName='__setitem__', 
                               cName='[](PyVector2DList &item, Index index, const py::object& vector) {\n            item.PySetItem(index, vector); }', 
                               description="set list item 'index' with data, write: data[index] = ...",
                               isLambdaFunction = True)

plr.DefPyFunctionAccess(cClass=classStr, pyName='__getitem__', 
                               cName='[](const PyVector2DList &item, Index index) {\n            return py::array_t<Real>(item[index].NumberOfItems(), item[index].GetDataPointer()); }', 
                               description="get copy of list item with 'index' as vector",
                               isLambdaFunction = True)

plr.DefPyFunctionAccess(cClass=classStr, pyName='__repr__', 
                               cName='[](const PyVector2DList &item) {\n            return EXUstd::ToString(item.GetPythonObject()); }', 
                               description="return the string representation of the Vector2DList data, e.g.: print(data)",
                               isLambdaFunction = True)

#++++++++++++++++
plr.DefPyFinishClass('PyVector2DList')

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#documentation and pybindings for PyVector6DList
classStr = 'PyVector6DList'
pyClassStr = 'Vector6DList'
plr.DefPyStartClass(classStr, pyClassStr, "The Vector6DList is used to represent lists of 6D vectors. This is used to transfer such lists from Python to C++." +
        ' \\\\ \\\\ Usage: \\bi\n'+
        '  \\item Create empty \\texttt{Vector6DList} with \\texttt{x = Vector6DList()} \n'+
        '  \\item Convert into list of numpy arrays: \\texttt{x.GetPythonObject()}\n'+
        '  \\item similar to Vector3DList !\n'+
        '\\ei\n', subSection=True)

plr.DefLatexStartTable(pyClassStr)

plr.sPy += '        .def(py::init<const py::object&>(), py::arg("listOfArrays"))\n' #constructor with numpy array or list of lists

plr.DefPyFunctionAccess(cClass=classStr, pyName='Append', cName='PyAppend', 
                               argList=['pyArray'],
                               description="add single array or list to Vector6DList; array or list must have appropriate dimension!")
                                                                                                            
plr.DefPyFunctionAccess(cClass=classStr, pyName='GetPythonObject', cName='GetPythonObject', 
                               description="convert Vector6DList into (copied) list of numpy arrays")

plr.DefPyFunctionAccess(cClass=classStr, pyName='__len__', 
                               cName='[](const PyVector6DList &item) {\n            return item.NumberOfItems(); }', 
                               description="return length of the Vector6DList, using len(data) where data is the Vector6DList",
                               isLambdaFunction = True)

plr.DefPyFunctionAccess(cClass=classStr, pyName='__setitem__', 
                               cName='[](PyVector6DList &item, Index index, const py::object& vector) {\n            item.PySetItem(index, vector); }', 
                               description="set list item 'index' with data, write: data[index] = ...",
                               isLambdaFunction = True)

plr.DefPyFunctionAccess(cClass=classStr, pyName='__getitem__', 
                               cName='[](const PyVector6DList &item, Index index) {\n            return py::array_t<Real>(item[index].NumberOfItems(), item[index].GetDataPointer()); }', 
                               description="get copy of list item with 'index' as vector",
                               isLambdaFunction = True)

plr.DefPyFunctionAccess(cClass=classStr, pyName='__repr__', 
                               cName='[](const PyVector6DList &item) {\n            return EXUstd::ToString(item.GetPythonObject()); }', 
                               description="return the string representation of the Vector6DList data, e.g.: print(data)",
                               isLambdaFunction = True)

#++++++++++++++++
plr.DefPyFinishClass('PyVector6DList')

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#documentation and pybindings for PyMatrix3DList
classStr = 'PyMatrix3DList'
pyClassStr = 'Matrix3DList'
plr.DefPyStartClass(classStr, pyClassStr, "The Matrix3DList is used to represent lists of 3D Matrices. . This is used to transfer such lists from Python to C++." +
        ' \\\\ \\\\ Usage: \\bi\n'+
        '  \\item Create empty \\texttt{Matrix3DList} with \\texttt{x = Matrix3DList()} \n'+
        '  \\item Create \\texttt{Matrix3DList} with list of numpy arrays:\\\\\\texttt{x = Matrix3DList([ numpy.eye(3), numpy.array([[1.,2.,3.],[4.,5.,6.],[7.,8.,9.]]) ])}\n'+
        # '  \\item Create \\texttt{Matrix3DList} with list of lists \\texttt{x = Matrix3DList([[1.,2.,3.], [4.,5.,6.]])}\n'+
        '  \\item Append item: \\texttt{x.Append(numpy.eye(3))}\n'+
        '  \\item Convert into list of numpy arrays: \\texttt{x.GetPythonObject()}\n'+
        '  \\item similar to Vector3DList !\n'+
        '\\ei\n', subSection=True)

plr.DefLatexStartTable(pyClassStr)

plr.sPy += '        .def(py::init<const py::object&>(), py::arg("listOfArrays"))\n' #constructor with numpy array or list of lists

plr.DefPyFunctionAccess(cClass=classStr, pyName='Append', cName='PyAppend', 
                               argList=['pyArray'],
                               description="add single 3D array or list of lists to Matrix3DList; array or lists must have appropriate dimension!")
                                                                                                            
plr.DefPyFunctionAccess(cClass=classStr, pyName='GetPythonObject', cName='GetPythonObject', 
                               description="convert Matrix3DList into (copied) list of 2D numpy arrays")

plr.DefPyFunctionAccess(cClass=classStr, pyName='__len__', 
                               cName='[](const PyMatrix3DList &item) {\n            return item.NumberOfItems(); }', 
                               description="return length of the Matrix3DList, using len(data) where data is the Matrix3DList",
                               isLambdaFunction = True)

plr.DefPyFunctionAccess(cClass=classStr, pyName='__setitem__', 
                               cName='[](PyMatrix3DList &item, Index index, const py::object& matrix) {\n            item.PySetItem(index, matrix); }', 
                               description="set list item 'index' with matrix, write: data[index] = ...",
                               isLambdaFunction = True)

plr.DefPyFunctionAccess(cClass=classStr, pyName='__getitem__', 
                               cName='[](const PyMatrix3DList &item, Index index) {\n            return item.PyGetItem(index); }', 
                               description="get copy of list item with 'index' as matrix",
                               isLambdaFunction = True)

plr.DefPyFunctionAccess(cClass=classStr, pyName='__repr__', 
                               cName='[](const PyMatrix3DList &item) {\n            return EXUstd::ToString(item.GetPythonObject()); }', 
                               description="return the string representation of the Matrix3DList data, e.g.: print(data)",
                               isLambdaFunction = True)

#++++++++++++++++
plr.DefPyFinishClass('PyMatrix3DList')

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#documentation and pybindings for PyMatrix6DList
classStr = 'PyMatrix6DList'
pyClassStr = 'Matrix6DList'
plr.DefPyStartClass(classStr, pyClassStr, "The Matrix6DList is used to represent lists of 6D Matrices. . This is used to transfer such lists from Python to C++." +
        ' \\\\ \\\\ Usage: \\bi\n'+
        '  \\item Create empty \\texttt{Matrix6DList} with \\texttt{x = Matrix6DList()} \n'+
        '  \\item Create \\texttt{Matrix6DList} with list of numpy arrays:\\\\\\texttt{x = Matrix6DList([ numpy.eye(6), 2*numpy.eye(6) ])}\n'+
        '  \\item Append item: \\texttt{x.Append(numpy.eye(6))}\n'+
        '  \\item Convert into list of numpy arrays: \\texttt{x.GetPythonObject()}\n'+
        '  \\item similar to Matrix3DList !\n'+
        '\\ei\n', subSection=True)

plr.DefLatexStartTable(pyClassStr)

plr.sPy += '        .def(py::init<const py::object&>(), py::arg("listOfArrays"))\n' #constructor with numpy array or list of lists

plr.DefPyFunctionAccess(cClass=classStr, pyName='Append', cName='PyAppend', 
                               argList=['pyArray'],
                               description="add single 6D array or list of lists to Matrix6DList; array or lists must have appropriate dimension!")
                                                                                                            
plr.DefPyFunctionAccess(cClass=classStr, pyName='GetPythonObject', cName='GetPythonObject', 
                               description="convert Matrix6DList into (copied) list of 2D numpy arrays")

plr.DefPyFunctionAccess(cClass=classStr, pyName='__len__', 
                               cName='[](const PyMatrix6DList &item) {\n            return item.NumberOfItems(); }', 
                               description="return length of the Matrix6DList, using len(data) where data is the Matrix6DList",
                               isLambdaFunction = True)

plr.DefPyFunctionAccess(cClass=classStr, pyName='__setitem__', 
                               cName='[](PyMatrix6DList &item, Index index, const py::object& matrix) {\n            item.PySetItem(index, matrix); }', 
                               description="set list item 'index' with matrix, write: data[index] = ...",
                               isLambdaFunction = True)

plr.DefPyFunctionAccess(cClass=classStr, pyName='__getitem__', 
                               cName='[](const PyMatrix6DList &item, Index index) {\n            return item.PyGetItem(index); }', 
                               description="get copy of list item with 'index' as matrix",
                               isLambdaFunction = True)

plr.DefPyFunctionAccess(cClass=classStr, pyName='__repr__', 
                               cName='[](const PyMatrix6DList &item) {\n            return EXUstd::ToString(item.GetPythonObject()); }', 
                               description="return the string representation of the Matrix6DList data, e.g.: print(data)",
                               isLambdaFunction = True)

#++++++++++++++++
plr.DefPyFinishClass('PyMatrix6DList')

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#now finalize files:
plr.CreateNewRSTfile('TypeDefinitions')
plr.sLatex += sLenum #put latex description of enums after the systemData section
plr.sRST += sRSTenum #put RST description of enums after the systemData section
plr.CreateNewRSTfile('') #this finalizes the list

directoryString = '../Autogenerated/'
pybindFile = directoryString + 'pybind_manual_classes.h'
latexFile = '../../../docs/theDoc/manual_interfaces.tex'

file=open(pybindFile,'w')  #clear file by one write access
file.write('// AUTO:  ++++++++++++++++++++++\n')
file.write('// AUTO:  pybind11 manual module includes; generated by Johannes Gerstmayr\n')
file.write('// AUTO:  last modified = '+ GetDateStr() + '\n')
file.write('// AUTO:  ++++++++++++++++++++++\n')
file.write(plr.PyStr())
file.close()

file=open(latexFile,'w')  #clear file by one write access
file.write('% ++++++++++++++++++++++\n')
file.write('% description of manual pybind interfaces; generated by Johannes Gerstmayr\n')
file.write('% ++++++++++++++++++++++\n')
file.write(plr.LatexStr())
file.close()

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#RST files
#rstFile = '../../../docs/RST/cInterface/exudyn.rst'
rstDir = '../../../docs/RST/cInterface/'
rstIndexFile = 'CInterfaceIndex.rst'
#create primary toc
indexRST = """
.. _sec-pcpp-command-interface:

============================
Python-C++ command interface
============================

.. toctree::
   :maxdepth: 3

"""
for (file, text) in plr.rstFileLists:
    indexRST += '   '+file+'\n'

    file=io.open(rstDir+file+'.rst','w',encoding='utf8')  #clear file by one write access
    file.write(text)
    #file.write(plr.RSTStr())
    file.close()

indexRST += '\n'


file=io.open(rstDir+rstIndexFile,'w',encoding='utf8')  #clear file by one write access
file.write(indexRST)
file.close()


