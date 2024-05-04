# -*- coding: utf-8 -*-
"""
Created on Fri May 18 08:53:30 2018

@author: Johannes Gerstmayr

automatically generate pybindings for specific classes and functions AND latex documentation for these functions
"""

#TODO:
#add citations
#add missing figures (items, ?check other replacements)

import io   #for utf-8 encoding
import copy

from exudynVersion import exudynVersionString

from autoGenerateHelper import PyLatexRST, GetDateStr, RSTlabelString
     #AddEnumValue, DefPyFunctionAccess, DefPyStartClass, DefPyFinishClass, DefLatexStartClass, DefLatexFinishTable

from autoGenerateHelper import localListFunctionNames, localListClassNames, localListEnumNames

theDocDir = '../../../docs/theDoc/'
rstDir='../../../docs/RST/'


localListFunctionNames.clear()
localListClassNames.clear()
localListEnumNames.clear()

#itemDict = 'dict'               #stub type for item classes (in fact convertable to dict)
itemDict = 'Any'                #stub type for item classes (in fact convertable to dict)
returnedArray = 'List[float]'   #stub type for for returned numpy array
listOrArray = 'List[float]'     #stub type for input as list or numpy array
vector2D = '[float,float]'#stub type for Vector3D
vector3D = '[float,float,float]'#stub type for Vector3D
vector6D = '[float,float,float,float,float,float]'#stub type for Vector6D

matrix3D = 'NDArray[Shape2D[3,3], float]'#stub type for Matrix3D
matrix6D = 'NDArray[Shape2D[6,6], float]'#stub type for Matrix6D

#for objects with trivial or implemented copy constructor:
pickleDictTemplate = """        .def(py::pickle(
            [](const {ClassName}& self) {
                return py::make_tuple(self.GetDictionary());
            },
            [](const py::tuple& t) {
                CHECKandTHROW(t.size() == 1, "{ClassName}: loading data with pickle received invalid data structure!");
                {ClassName} self;
                self.SetDictionary(py::cast<py::dict>(t[0]));
                return self;
            }))
"""
#for objects which cannot be copied:
pickleDictTemplateNew = """        .def(py::pickle(
            [](const {ClassName}& self) {
                return py::make_tuple(self.GetDictionary());
            },
            [](const py::tuple& t) {
                CHECKandTHROW(t.size() == 1, "{ClassName}: loading data with pickle received invalid data structure!");
                {ClassName}* self = new {ClassName}();
                self->SetDictionary(py::cast<py::dict>(t[0]));
                return self;
            }))
"""


# s = ''  #C++ pybind local includes
# sL = '' #Latex documentation
# sR = '' #RST documentation
rstList = [] #list of tuples with (file, text) => add lists, clean content (examples, lists, etc.)

#
#enumExportValues = '.export_values()' #don't do that for enum class => would be visible at global scope
enumExportValues = '' #since 1.6.98

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
plrmain.AddDocuCodeBlock(code="""
import exudyn as exu
""")

#plrmain.AddDocuList(itemList=['\\texttt{import exudyn as exu}'], itemText='[]')
plrmain.AddDocu('For compatibility with examples and other users, we recommend to use the \\texttt{exu} abbreviation throughout. '+
                'In addition, you may work with a convenient interface for your items, therefore also always include:')
plrmain.AddDocuCodeBlock(code="""
from exudyn.itemInterface import *
""")
#plrmain.AddDocuList(itemList=['\\texttt{from exudyn.itemInterface import *}'], itemText='[]')
plrmain.AddDocu('Note that including \\texttt{exudyn.utilities} will cover \\texttt{itemInterface}. '+
                'Also note that \\texttt{from ... import *} is not recommended in general and it will not work in certain cases, '+
                'e.g., if you like to compute on a cluster. However, it greatly simplifies life for smaller models and you may replace '+
                'imports in your files afterwards by removing the star import.')

plrmain.AddDocu('The general hub to multibody dynamics models is provided by the classes \\texttt{SystemContainer} and \\texttt{MainSystem}, '+
                'except for some very basic system functionality (which is inside the \\codeName\\ module). \n\n'+
                'You can create a new \\texttt{SystemContainer}, which is a class that is initialized by assigning a '+
                'system container to a variable, usually denoted as \\texttt{SC}:')
plrmain.AddDocuCodeBlock(code="""
SC = exu.SystemContainer()
""")
#plrmain.AddDocuList(itemList=['\\texttt{SC = exu.SystemContainer()}'], itemText='[]')
plrmain.AddDocu('Note that creating a second \\texttt{exu.SystemContainer()} will be independent of \\texttt{SC} and therefore makes no sense if you do not intend to work with two different containers.\n')

plrmain.AddDocu('To add a MainSystem to system container \\texttt{SC} and store as variable \\texttt{mbs}, write:')

plrmain.AddDocuCodeBlock(code="""
mbs = SC.AddSystem()
""")
#plrmain.AddDocuList(itemList=['\\texttt{mbs = SC.AddSystem()}'], itemText='[]')

plrmain.AddDocu('Furthermore, there are a couple of commands available directly in the \\texttt{exudyn} module, given in the following subsections. '+
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
#  delete reference to mbs and mbs2 (usually not necessary):
del mbs, mbs2
#  reset system container (mbs becomes invalid):
SC.Reset()
""")
plrmain.AddDocu('If you run a parameter variation (check \\texttt{Examples/parameterVariationExample.py}), '+
                'you may reset or delete the created \\texttt{MainSystem} \\texttt{mbs} and '+
                'the \\texttt{SystemContainer} \\texttt{SC} before creating new instances in order to avoid memory growth.')

#+++++++++++++++++++++++++++++++++++
#ITEMINDEX
plrmain.AddDocu('Many functions will work with node numbers (\\texttt{NodeIndex}), object numbers (\\texttt{ObjectIndex}),'+
                'marker numbers (\\texttt{MarkerIndex}) and others. These numbers are special Python objects, which have been '+
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
There are only a few very important cases where data is referenced in \\codeName\\ , the main ones are 
\\texttt{SystemContainer}, 
\\texttt{MainSystem}, 
\\texttt{VisualizationSettings}, and
\\texttt{SimulationSettings} which are always references to internal C++ classes.
The following code snippets and comments should explain this behavior:
""", section='Copying and referencing C++ objects', sectionLevel=2, sectionLabel='sec:generalPythonInterface:copyref')

plrmain.AddDocuCodeBlock(code="""
import copy                        #for real copying
import exudyn as exu
from exudyn.utilities import *
#create system container, referenced from SC:
SC = exu.SystemContainer()
SC2 = SC                           #this will only put a reference to SC
                                   #SC2 and SC represent the SAME C++ object
#add a MainSystem (multibody system):
mbs = SC.AddSystem()               #get reference mbs to C++ system
mbs2=mbs                           #again, mbs2 and mbs refer to the same C++ object
og = mbs.AddObject(ObjectGround()) #copy data of ObjectGround() into C++
o0 = mbs.GetObject(0)              #get copy of internal data as dictionary

mbsCopy=copy.copy(mbs)             #mbsCopy is now a real copy of mbs; uses pickle; experimental!
SC.Append(mbsCopy)                 #this is needed to work with mbsCopy

del o0                             #delete the local dictionary; C++ data not affected
del mbs, mbs2                      #references to mbs deleted (C++ data still available)
del mbsCopy                        #now also copy of mbs destroyed
del SC                             #references to SystemContainer deleted
#at this point, mbs and SC are not available any more (data will be cleaned up by Python)
""")

#+++++++++++++++++++++++++++++++++++
#EXCEPTIONS
plrmain.AddDocu('There are several levels of type and argument checks, leading to different types of errors and exceptions. '+
                'The according error messages are non-unique, because they may be raised in Python modules or in C++, '+
                'and they may be raised on different levels of the code. Error messages depend on Python version '+
                'and on your iPython console. Very often the exception may be called \\texttt{ValueError}, but it must'+
                'not mean that it is a wrong error, but it could also be, e.g., a wrong order of function calls.',
                section='Exceptions and Error Messages', sectionLevel=2,sectionLabel='sec:cinterface:exceptions')

plrmain.AddDocu("As an example, a type conversion error is raised when providing wrong argument types, e.g., try \\texttt{exu.GetVersionString('abc')}:")

plrmain.AddDocuCodeBlock(code="""
Traceback (most recent call last):

File "C:\\Users\\username\\AppData\\Local\\Temp\\ipykernel_24988\\2212168679.py", line 1, in <module>
    exu.GetVersionString('abc')

TypeError: GetVersionString(): incompatible function arguments. The following argument types are supported:
    1. (addDetails: bool = False) -> str

Invoked with: 'abc'
""",
pythonStyle=False)

plrmain.AddDocu('Note that your particular error message may be different.')
plrmain.AddDocu('Another error results from internal type and range checking, saying User ERROR, '+
                'as it is due to a wrong input of the user. For this, we try')

plrmain.AddDocuCodeBlock(code="mbs.AddObject('abc')")

plrmain.AddDocu('Which results in an error message similar to:')
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

#plr.sPy +=	'  py::enum_<' + pyClass + '>(m, "' + pyClass + '")\n'
plr.DefStartEnumClass(className = pyClass, 
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
plr.AddEnumValue(pyClass, 'AccelerationLocal', 'measure (translational) acceleration of node or object in local coordinates')

plr.AddEnumValue(pyClass, 'RotationMatrix', 'measure rotation matrix of rigid body node or object')
plr.AddEnumValue(pyClass, 'Rotation', 'measure, e.g., scalar rotation of 2D body, Euler angles of a 3D object or rotation within a joint')
plr.AddEnumValue(pyClass, 'AngularVelocity', 'measure angular velocity of node or object')
plr.AddEnumValue(pyClass, 'AngularVelocityLocal', 'measure local (body-fixed) angular velocity of node or object')
plr.AddEnumValue(pyClass, 'AngularAcceleration', 'measure angular acceleration of node or object')
plr.AddEnumValue(pyClass, 'AngularAccelerationLocal', 'measure angular acceleration of node or object in local coordinates')

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

plr.sPy +=	'		'+enumExportValues+';\n\n'
plr.DefLatexFinishTable()

#+++++++++++++++++++++++++++++++++++++++++++++++++++
pyClass = 'ConfigurationType'

descriptionStr = 'This section shows the ' + pyClass + ' structure, which is used for selecting a configuration for reading or writing information to the module. Specifically, the ConfigurationType.Current configuration is usually used at the end of a solution process, to obtain result values, or the ConfigurationType.Initial is used to set initial values for a solution process.\n\n'

# plr.sPy +=	'  py::enum_<' + pyClass + '>(m, "' + pyClass + '")\n'
plr.DefStartEnumClass(className = pyClass, 
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

plr.sPy +=	'		'+enumExportValues+';\n\n'
plr.DefLatexFinishTable()

#+++++++++++++++++++++++++++++++++++++++++++++++++++
pyClass = 'ItemType'

descriptionStr = 'This section shows the ' + pyClass + ' structure, which is used for defining types of indices, e.g., in render window and will be also used in item dictionaries in future.\n\n'

# plr.sPy +=	'  py::enum_<' + pyClass + '>(m, "' + pyClass + '")\n'
plr.DefStartEnumClass(className = pyClass, 
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

plr.sPy +=	'		'+enumExportValues+';\n\n'
plr.DefLatexFinishTable()

#+++++++++++++++++++++++++++++++++++++++++++++++++++
pyClass = 'NodeType'
cClass = 'Node'

descriptionStr = 'This section shows the ' + pyClass + ' structure, which is used for defining node types for 3D rigid bodies.\n\n'

# plr.sPy +=	'  py::enum_<' + cClass + '::Type' + '>(m, "' + pyClass + '")\n'
plr.DefStartEnumClass(className = pyClass, 
                            description=descriptionStr, 
                            subSection=True, labelName='sec:'+pyClass, cClass=cClass + '::Type')
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
#plr.AddEnumValue(cClass, 'LieGroupWithDataCoordinates', 'node to be solved with Lie group methods, having data coordinates')
plr.AddEnumValue(cClass, 'GenericODE2', 'node with general ODE2 variables')
plr.AddEnumValue(cClass, 'GenericODE1', 'node with general ODE1 variables')
plr.AddEnumValue(cClass, 'GenericAE', 'node with general algebraic variables')
plr.AddEnumValue(cClass, 'GenericData', 'node with general data variables')
plr.AddEnumValue(cClass, 'PointSlope1', 'node with 1 slope vector')
plr.AddEnumValue(cClass, 'PointSlope12', 'node with 2 slope vectors in x and y direction')
plr.AddEnumValue(cClass, 'PointSlope23', 'node with 2 slope vectors in y and z direction')


plr.sPy +=	'		'+enumExportValues+';\n\n'
plr.DefLatexFinishTable()

#+++++++++++++++++++++++++++++++++++++++++++++++++++
pyClass = 'JointType'
cClass = 'Joint'

descriptionStr = 'This section shows the ' + pyClass + ' structure, which is used for defining joint types, used in KinematicTree.\n\n'

# plr.sPy +=	'  py::enum_<' + cClass + '::Type' + '>(m, "' + pyClass + '")\n'
plr.DefStartEnumClass(className = pyClass, 
                      description=descriptionStr, 
                      subSection=True, labelName='sec:'+pyClass, cClass=cClass + '::Type')
plr.DefLatexStartTable(pyClass)
#keep this list synchronized with the accoring enum structure in C++!!!
plr.AddEnumValue(cClass, '_None', 'node has no type')

plr.AddEnumValue(cClass, 'RevoluteX', 'revolute joint type with rotation around local X axis')
plr.AddEnumValue(cClass, 'RevoluteY', 'revolute joint type with rotation around local Y axis')
plr.AddEnumValue(cClass, 'RevoluteZ', 'revolute joint type with rotation around local Z axis')
plr.AddEnumValue(cClass, 'PrismaticX', 'prismatic joint type with translation along local X axis')
plr.AddEnumValue(cClass, 'PrismaticY', 'prismatic joint type with translation along local Y axis')
plr.AddEnumValue(cClass, 'PrismaticZ', 'prismatic joint type with translation along local Z axis')

plr.sPy +=	'		'+enumExportValues+';\n\n'
plr.DefLatexFinishTable()

#+++++++++++++++++++++++++++++++++++++++++++++++++++
pyClass = 'DynamicSolverType'

descriptionStr = 'This section shows the ' + pyClass + ' structure, which is used for selecting dynamic solvers for simulation.\n\n'

# plr.sPy +=	'  py::enum_<' + pyClass + '>(m, "' + pyClass + '")\n'
plr.DefStartEnumClass(className = pyClass, 
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

plr.sPy +=	'		'+enumExportValues+';\n\n'
plr.DefLatexFinishTable()

#+++++++++++++++++++++++++++++++++++++++++++++++++++
pyClass = 'CrossSectionType'

descriptionStr = 'This section shows the ' + pyClass + ' structure, which is used for defining beam cross section types.\n\n'

# plr.sPy +=	'  py::enum_<' + pyClass + '>(m, "' + pyClass + '")\n'
plr.DefStartEnumClass(className = pyClass, 
                        description=descriptionStr, 
                        subSection=True, labelName='sec:'+pyClass)
plr.DefLatexStartTable(pyClass)
#keep this list synchronized with the accoring enum structure in C++!!!
plr.AddEnumValue(pyClass, 'Polygon', 'cross section profile defined by polygon')
plr.AddEnumValue(pyClass, 'Circular', 'cross section is circle or elliptic')

plr.sPy +=	'		'+enumExportValues+';\n\n'
plr.DefLatexFinishTable()

#+++++++++++++++++++++++++++++++++++++++++++++++++++
pyClass = 'KeyCode'

descriptionStr = 'This section shows the ' + pyClass + ' structure, which is used for special key codes in keyPressUserFunction.\n\n'

# plr.sPy +=	'  py::enum_<' + pyClass + '>(m, "' + pyClass + '")\n'
plr.DefStartEnumClass(className = pyClass, 
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

plr.sPy +=	'		'+enumExportValues+';\n\n'
plr.DefLatexFinishTable()

#+++++++++++++++++++++++++++++++++++++++++++++++++++
pyClass = 'LinearSolverType'


descriptionStr = 'This section shows the ' + pyClass + ' structure, which is used for selecting linear solver types, which are dense or sparse solvers.\n\n'

# plr.sPy +=	'  py::enum_<' + pyClass + '>(m, "' + pyClass + '")\n'
plr.DefStartEnumClass(className = pyClass, 
                            description=descriptionStr, 
                            subSection=True, labelName='sec:'+pyClass)
plr.DefLatexStartTable(pyClass)
#keep this list synchronized with the accoring enum structure in C++!!!
plr.AddEnumValue(pyClass, '_None', 'no value; used, e.g., if no solver is selected')
plr.AddEnumValue(pyClass, 'EXUdense', 'use dense matrices and according solvers for densly populated matrices (usually the CPU time grows cubically with the number of unknowns)')
plr.AddEnumValue(pyClass, 'EigenSparse', 'use sparse matrices and according solvers; additional overhead for very small multibody systems; specifically, memory allocation is performed during a factorization process')
plr.AddEnumValue(pyClass, 'EigenSparseSymmetric', 'use sparse matrices and according solvers; NOTE: this is the symmetric mode, which assumes symmetric system matrices; this is EXPERIMENTAL and should only be used of user knows that the system matrices are (nearly) symmetric; does not work with scaled GeneralizedAlpha matrices; does not work with constraints, as it must be symmetric positive definite')
plr.AddEnumValue(pyClass, 'EigenDense', "use Eigen's LU factorization with partial pivoting (faster than EXUdense) or full pivot (if linearSolverSettings.ignoreSingularJacobian=True; is much slower)")

plr.sPy +=	'		'+enumExportValues+';\n\n'
plr.DefLatexFinishTable()


#+++++++++++++++++++++++++++++++++++++++++++++++++++
pyClass = 'ContactTypeIndex'
cClass = 'Contact'

descriptionStr = 'This section shows the ' + pyClass + ' structure, which is in GeneralContact to select specific contact items, such as spheres, ANCFCable or triangle items.\n\n'

# plr.sPy +=	'  py::enum_<' + cClass + '::TypeIndex' + '>(m, "' + pyClass + '")\n'
plr.DefStartEnumClass(className = pyClass, 
                            description=descriptionStr, 
                            subSection=True, labelName='sec:'+pyClass, cClass=cClass + '::TypeIndex')
plr.DefLatexStartTable(pyClass)
#keep this list synchronized with the accoring enum structure in C++!!!
plr.AddEnumValue(cClass, 'IndexSpheresMarkerBased', 'spheres attached to markers')
plr.AddEnumValue(cClass, 'IndexANCFCable2D', 'ANCFCable2D contact items')
plr.AddEnumValue(cClass, 'IndexTrigsRigidBodyBased', 'triangles attached to rigid body (or rigid body marker)')
plr.AddEnumValue(cClass, 'IndexEndOfEnumList', 'signals end of list')

plr.sPy +=	'		'+enumExportValues+';\n\n'
plr.DefLatexFinishTable()

sStubEnums = plr.sPyi
plr.sPyi = ''
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

savedPyi = '' #to reverse order

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
                               returnType='str',
                               )

plr.DefPyFunctionAccess('', 'Help', 'PyHelp', 
                               description='Show basic help information',
                               returnType='None',
                               )

sOld = plr.PyStr()
plr.DefPyFunctionAccess('', 'RequireVersion', '', 
                               argList=['requiredVersionString'],
                               description = 'Checks if the installed version is according to the required version. Major, micro and minor version must agree the required level. This function is defined in the \\texttt{__init__.py} file', 
                               example='exu.RequireVersion("1.0.31")',
                               argTypes=['str'],
                               returnType='None',
                               )
plr.sPy = sOld #this function is defined in __init__.py ==> do not add to cpp bindings

plr.DefPyFunctionAccess(cClass='', pyName='StartRenderer', cName='PyStartOpenGLRenderer', 
                                defaultArgs=['0'],
                                argList=['verbose'],
                                description="Start OpenGL rendering engine (in separate thread) for visualization of rigid or flexible multibody system; use verbose=1 to output information during OpenGL window creation; verbose=2 produces more output and verbose=3 gives a debug level; some of the information will only be seen in windows command (powershell) windows or linux shell, but not inside iPython of e.g. Spyder",
                                returnType='bool',
                                )

#new, defined in C++ as lambda function:
sOld = plr.PyStr()
plr.DefPyFunctionAccess('', 'StopRenderer', 'no direct link to C++ here', "Stop OpenGL rendering engine")
plr.sPy = sOld

plr.DefPyFunctionAccess(cClass='', pyName='IsRendererActive', cName='PyIsRendererActive', 
                                description="returns True if GLFW renderer is available and running; otherwise False",
                                returnType='bool',
                                )

plr.DefPyFunctionAccess(cClass='', pyName='DoRendererIdleTasks', cName='PyDoRendererIdleTasks', 
                                defaultArgs=['0'],
                                argList=['waitSeconds'],
                                description="Call this function in order to interact with Renderer window; use waitSeconds in order to run this idle tasks while animating a model (e.g. waitSeconds=0.04), use waitSeconds=0 without waiting, or use waitSeconds=-1 to wait until window is closed",
                                returnType='None',
                                )

sOld = plr.PyStr()
plr.DefPyFunctionAccess(cClass='', pyName='SolveStatic', cName='SolveDynamic', 
                               description='Static solver function, mapped from module \\texttt{solver}, to solve static equations (without inertia terms) of constrained rigid or flexible multibody system; for details on the Python interface see \\refSection{sec:mainsystemextensions:SolveStatic}; for background on solvers, see \\refSection{sec:solvers}',
                               argList=['mbs', 'simulationSettings', 'updateInitialValues', 'storeSolver'],
                               defaultArgs=['','exudyn.SimulationSettings()','False','True'],
                               argTypes=['MainSystem','SimulationSettings', '', ''],
                               returnType='bool',
                               )
                
plr.DefPyFunctionAccess(cClass='', pyName='SolveDynamic', cName='SolveDynamic', 
                               description='Dynamic solver function, mapped from module \\texttt{solver}, to solve equations of motion of constrained rigid or flexible multibody system; for details on the Python interface see \\refSection{sec:mainsystemextensions:SolveDynamic}; for background on solvers, see \\refSection{sec:solvers}',
                               argList=['mbs', 'simulationSettings', 'solverType', 'updateInitialValues', 'storeSolver'],
                               defaultArgs=['','exudyn.SimulationSettings()','exudyn.DynamicSolverType.GeneralizedAlpha','False','True'],
                               argTypes=['MainSystem','SimulationSettings', 'DynamicSolverType', '', ''],
                               returnType='bool',
                               )
                
plr.DefPyFunctionAccess(cClass='', pyName='ComputeODE2Eigenvalues', cName='ComputeODE2Eigenvalues', 
                               description='Simple interface to scipy eigenvalue solver for eigenvalue analysis of the second order differential equations part in mbs, mapped from module \\texttt{solver}; for details on the Python interface see \\refSection{sec:mainsystemextensions:ComputeODE2Eigenvalues}',
                               argList=['mbs', 'simulationSettings', 'useSparseSolver', 'numberOfEigenvalues', 'setInitialValues', 'convert2Frequencies'],
                               defaultArgs=['','exudyn.SimulationSettings()','False','-1','True','False'],
                               #argTypes=['MainSystem','SimulationSettings', 'bool', 'int', 'bool', 'bool'],
                               argTypes=['MainSystem','SimulationSettings', '', '', '', ''],
                               returnType='bool',
                               )
plr.sPy = sOld

plr.DefPyFunctionAccess(cClass='', pyName='SetOutputPrecision', cName='PySetOutputPrecision', 
                                description="Set the precision (integer) for floating point numbers written to console (reset when simulation is started!); NOTE: this affects only floats converted to strings inside C++ exudyn; if you print a float from Python, it is usually printed with 16 digits; if printing numpy arrays, 8 digits are used as standard, to be changed with numpy.set_printoptions(precision=16); alternatively convert into a list",
                                argList=['numberOfDigits'],
                                argTypes=['int'],
                                returnType='None',
                                )

plr.DefPyFunctionAccess(cClass='', pyName='SetLinalgOutputFormatPython', cName='PySetLinalgOutputFormatPython', 
                                description="True: use Python format for output of vectors and matrices; False: use matlab format",
                                argList=['flagPythonFormat'],
                                argTypes=['bool'],
                                returnType='None',
                                )

plr.DefPyFunctionAccess(cClass='', pyName='SetWriteToConsole', cName='PySetWriteToConsole', 
                            description="set flag to write (True) or not write to console; default = True",
                            argList=['flag'],
                            argTypes=['bool'],
                            returnType='None',
                            )

print('complete stub file for exudyn module')

plr.DefPyFunctionAccess(cClass='', pyName='SetWriteToFile', cName='PySetWriteToFile', 
                            description="set flag to write (True) or not write to console; default value of flagWriteToFile = False; flagAppend appends output to file, if set True; in order to finalize the file, write \\texttt{exu.SetWriteToFile('', False)} to close the output file",
                            argList=['filename', 'flagWriteToFile', 'flagAppend'],
                            defaultArgs=['', 'True', 'False'],
                            example="exu.SetWriteToConsole(False) #no output to console\\\\exu.SetWriteToFile(filename='testOutput.log', flagWriteToFile=True, flagAppend=False)\\\\exu.Print('print this to file')\\\\exu.SetWriteToFile('', False) #terminate writing to file which closes the file",
                            argTypes=['str','',''],
                            returnType='None',
                            )

plr.DefPyFunctionAccess(cClass='', pyName='SetPrintDelayMilliSeconds', cName='PySetPrintDelayMilliSeconds', 
                            description="add some delay (in milliSeconds) to printing to console, in order to let Spyder process the output; default = 0",
                            argList=['delayMilliSeconds'],
                            argTypes=['int'],
                            returnType='None',
                            )

plr.DefPyFunctionAccess(cClass='', pyName='Print', cName='PyPrint', 
                            description="this allows printing via exudyn with similar syntax as in Python print(args) except for keyword arguments: print('test=',42); allows to redirect all output to file given by SetWriteToFile(...); does not output in case that SetWriteToConsole is set to False",
                            argList=['*args'], 
                            argTypes=['Any'],
                            returnType='None',
                            )

plr.DefPyFunctionAccess(cClass='', pyName='SuppressWarnings', cName='PySuppressWarnings', 
                            description="set flag to suppress (=True) or enable (=False) warnings",
                            argList=['flag'],
                            argTypes=['bool'],
                            returnType='None',
                            )

plr.DefPyFunctionAccess(cClass='', pyName='InfoStat', cName='PythonInfoStat', 
                            description='Retrieve list of global information on memory allocation and other counts as list:[array_new_counts, array_delete_counts, vector_new_counts, vector_delete_counts, matrix_new_counts, matrix_delete_counts, linkedDataVectorCast_counts]; May be extended in future; if writeOutput==True, it additionally prints the statistics; counts for new vectors and matrices should not depend on numberOfSteps, except for some objects such as ObjectGenericODE2 and for (sensor) output to files; Not available if code is compiled with __FAST_EXUDYN_LINALG flag',
                            argList=['writeOutput'],
                            defaultArgs=['True'],
                            argTypes=[''],
                            returnType='List[int]',
                            )

plr.DefPyFunctionAccess('', 'Go', 'PythonGo', 'Creates a SystemContainer SC and a main multibody system mbs',
                            returnType='None',
                            )

sPyOld = plr.PyStr() #Demos added via Python
plr.DefPyFunctionAccess(cClass='', pyName='Demo1', cName='Demo1', 
                            description="Run simple demo without graphics to check functionality, see exudyn/demos.py",
                            argList=['showAll'], 
                            argTypes=['bool'],
                            returnType='[MainSystem, SystemContainer]',
                            )
    
plr.DefPyFunctionAccess(cClass='', pyName='Demo2', cName='Demo2', 
                            description="Run advanced demo without graphics to check functionality, see exudyn/demos.py",
                            argList=['showAll'], 
                            argTypes=['bool'],
                            returnType='[MainSystem, SystemContainer]',
                            )
plr.sPy = sPyOld  #system container manually added 
    
plr.DefPyFunctionAccess('', 'InvalidIndex', 'GetInvalidIndex', 
                            "This function provides the invalid index, which may depend on the kind of 32-bit, 64-bit signed or unsigned integer; e.g. node index or item index in list; currently, the InvalidIndex() gives -1, but it may be changed in future versions, therefore you should use this function",
                            returnType='int',
                            )

plr.DefLatexDataAccess('__version__','stores the current version of the Exudyn package',
                       dataType='str', isTopLevel = True,
                       )

#plr.sPy += '        m.attr("symbolic") = symbolic;\n' 
plr.DefLatexDataAccess('symbolic','the symbolic submodule for creating symbolic variables in Python, see documentation of Symbolic; For details, see Section Symbolic.',
                       dataType='', isTopLevel = True,
                       )

plr.sPy += '        m.attr("experimental") = py::cast(&pyExperimental);\n' 
plr.DefLatexDataAccess('experimental','Experimental features, not intended for regular users; for available features, see the C++ code class PyExperimental',
                       dataType='Experimental', isTopLevel = True)

plr.sPy += '        m.attr("special") = py::cast(&pySpecial);\n' 
plr.DefLatexDataAccess('special','special attributes and functions, such as global (solver) flags or helper functions; not intended for regular users; for available features, see the C++ code class PySpecial',
                        dataType='Special', isTopLevel = True)

plr.DefLatexDataAccess('special.solver','special solver attributes and functions; not intended for regular users; for available features, see the C++ code class PySpecialSolver',
                        dataType='SpecialSolver', isTopLevel = True)

plr.DefLatexDataAccess('special.solver.timeout','if >= 0, the solver stops after reaching accoring CPU time specified with timeout; makes sense for parameter variation, automatic testing or for long-running simulations; default=-1 (no timeout)',
                        dataType='float', isTopLevel = True)


plr.sPy += '        m.attr("variables") = exudynVariables;\n' 
plr.DefLatexDataAccess('variables','this dictionary may be used by the user to store exudyn-wide data in order to avoid global Python variables; usage: exu.variables["myvar"] = 42 ',
                       dataType='dict', isTopLevel = True)

plr.sPy += '        m.attr("sys") = exudynSystemVariables;\n' 
plr.DefLatexDataAccess('sys',"this dictionary is used and reserved by the system, e.g. for testsuite, graphics or system function to store module-wide data in order to avoid global Python variables; the variable exu.sys['renderState'] contains the last render state after exu.StopRenderer() and can be used for subsequent simulations ",
                       dataType='dict', isTopLevel = True)


plr.DefPyFinishClass('')


savedPyi = plr.sPyi+savedPyi
plr.sPyi = ''




#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#currently, only latex + RST binding:
plr.CreateNewRSTfile('SystemContainer')
pyClassStr = 'SystemContainer'
classStr = 'Main'+pyClassStr
sPyOld = plr.PyStr() #systemcontainer manually added in C++

plr.DefPyStartClass(classStr, pyClassStr, '')

plr.AddDocu('The SystemContainer is the top level of structures in \\codeName. '+
            'The container holds all (multibody) systems, solvers and all other data structures for computation. '+
            "A SystemContainer is created by \\texttt{SC = exu.SystemContainer()}, understanding \\texttt{exu.SystemContainer} as a class like Python's internal list class, creating a list instance with \\texttt{x=list()}. "+
            'Currently, only one container shall be used. In future, multiple containers might be usable at the same time. '+
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
                        description="delete all multibody systems and reset SystemContainer (including graphics); this also releases SystemContainer from the renderer, which requires SC.AttachToRenderEngine() to be called in order to reconnect to rendering; a safer way is to delete the current SystemContainer and create a new one (SC=SystemContainer() )",
                        returnType='None',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='AddSystem', cName='AddMainSystem', 
                        description="add a new computational system", 
                        options='py::return_value_policy::reference',
                        returnType='MainSystem',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='Append', cName='AppendMainSystem', 
                        description="append an exsiting computational system to the system container; returns the number of MainSystem in system container", options='py::return_value_policy::reference',
                        argList=['mainSystem'],
                        argTypes=['MainSystem'],
                        returnType='int',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='NumberOfSystems', cName='NumberOfSystems', 
                        description="obtain number of multibody systems available in system container",
                        returnType='int',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetSystem', cName='GetMainSystem', 
                        description="obtain multibody systems with index from system container",
                        argList=['systemNumber'],
                        argTypes=['int'],
                        returnType='MainSystem',
                        )

#plr.sPy += '        .def_property("visualizationSettings", &MainSystemContainer::PyGetVisualizationSettings, &MainSystemContainer::PySetVisualizationSettings)\n' 
plr.DefLatexDataAccess('visualizationSettings','this structure is read/writeable and contains visualization settings, which are immediately applied to the rendering window. \\tabnewline\n    EXAMPLE:\\tabnewline\n    SC = exu.SystemContainer()\\tabnewline\n    SC.visualizationSettings.autoFitScene=False  ',
                       dataType = 'VisualizationSettings')

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetDictionary', cName='GetDictionary', 
                        description="[UNDER DEVELOPMENT]: return the dictionary of the system container data, e.g., to copy the system or for pickling",
                        argList=[],
                        argTypes=[],
                        returnType='dict',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetDictionary', cName='SetDictionary', 
                        description="[UNDER DEVELOPMENT]: set system container data from given dictionary; used for pickling",
                        argList=['systemDict'],
                        argTypes=['dict'],
                        returnType='None',
                        )


plr.DefPyFunctionAccess(cClass=classStr, pyName='GetRenderState', cName='PyGetRenderState', 
                        description="Get dictionary with current render state (openGL zoom, modelview, etc.); will have no effect if GLFW_GRAPHICS is deactivated",
                        example = "SC = exu.SystemContainer()\\\\renderState = SC.GetRenderState() \\\\print(renderState['zoom'])",
                        returnType='dict',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetRenderState', cName='PySetRenderState', 
                        description="Set current render state (openGL zoom, modelview, etc.) with given dictionary; usually, this dictionary has been obtained with GetRenderState; will have no effect if GLFW_GRAPHICS is deactivated",
                        example = "SC = exu.SystemContainer()\\\\SC.SetRenderState(renderState)",
                        argList=['renderState'],
                        argTypes=['dict'],
                        returnType='None',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='RedrawAndSaveImage', cName='RedrawAndSaveImage', 
                        description="Redraw openGL scene and save image (command waits until process is finished)",
                        returnType='None',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='WaitForRenderEngineStopFlag', cName='WaitForRenderEngineStopFlag', 
                        description="Wait for user to stop render engine (Press 'Q' or Escape-key); this command is used to have active response of the render window, e.g., to open the visualization dialog or use the right-mouse-button; behaves similar as mbs.WaitForUserToContinue()",
                        returnType='bool',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='RenderEngineZoomAll', cName='PyZoomAll', 
                        description="Send zoom all signal, which will perform zoom all at next redraw request",
                        returnType='None',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='AttachToRenderEngine', cName='AttachToRenderEngine', 
                        description="Links the SystemContainer to the render engine, such that the changes in the graphics structure drawn upon updates, etc.; done automatically on creation of SystemContainer; return False, if no renderer exists (e.g., compiled without GLFW) or cannot be linked (if other SystemContainer already linked)",
                        returnType='bool',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='DetachFromRenderEngine', cName='DetachFromRenderEngine', 
                        description="Releases the SystemContainer from the render engine; return True if successfully released, False if no GLFW available or detaching failed",
                        returnType='bool',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SendRedrawSignal', cName='SendRedrawSignal', 
                        description="This function is used to send a signal to the renderer that all MainSystems (mbs) shall be redrawn",
                        returnType='None',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetCurrentMouseCoordinates', cName='PyGetCurrentMouseCoordinates', 
                        description="Get current mouse coordinates as list [x, y]; x and y being floats, as returned by GLFW, measured from top left corner of window; use GetCurrentMouseCoordinates(useOpenGLcoordinates=True) to obtain OpenGLcoordinates of projected plane",
                        argList=['useOpenGLcoordinates'],
                        argTypes=['bool'],
                        defaultArgs=['False'],
                        returnType='[float,float]',
                        )
plr.sPy = sPyOld  #system container manually added 


plr.DefLatexFinishTable()#only finalize latex table

savedPyi = plr.sPyi+savedPyi
plr.sPyi = ''




#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
plr.CreateNewRSTfile('MainSystem')
classStr = 'MainSystem'
plr.DefPyStartClass(classStr, classStr, '', forbidPythonConstructor=False)

plr.AddDocu("This is the class which defines a (multibody) system. "+
            "The MainSystem shall only be created by \\texttt{SC.AddSystem()}, not with \\texttt{exu.MainSystem()}, as the latter one would not be linked to a SystemContainer. In some cases, you may use SC.Append(mbs). "+
            "In C++, there is a MainSystem (the part which links to Python) and a System (computational part). "+
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
                        description="assemble items (nodes, bodies, markers, loads, ...) of multibody system; Calls CheckSystemIntegrity(...), AssembleCoordinates(), AssembleLTGLists(), AssembleInitializeSystemCoordinates(), and AssembleSystemInitialize()",
                        returnType='None',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='AssembleCoordinates', cName='AssembleCoordinates', 
                        description="assemble coordinates: assign computational coordinates to nodes and constraints (algebraic variables)",
                        returnType='None',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='AssembleLTGLists', cName='AssembleLTGLists', 
                        description="build \\ac{LTG} coordinate lists for objects (used to build global ODE2RHS, MassMatrix, etc. vectors and matrices) and store special object lists (body, connector, constraint, ...)",
                        returnType='None',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='AssembleInitializeSystemCoordinates', cName='AssembleInitializeSystemCoordinates', 
                        description="initialize all system-wide coordinates based on initial values given in nodes",
                        returnType='None',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='AssembleSystemInitialize', cName='AssembleSystemInitialize', 
                        description="initialize some system data, e.g., generalContact objects (searchTree, etc.)",
                        returnType='None',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='Reset', cName='Reset', 
                        description="reset all lists of items (nodes, bodies, markers, loads, ...) and temporary vectors; deallocate memory",
                        returnType='None',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetSystemContainer', cName='GetMainSystemContainer', 
                        description="return the systemContainer where the mainSystem (mbs) was created",
                        returnType='SystemContainer',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='WaitForUserToContinue', cName='WaitForUserToContinue', 
                        description="interrupt further computation until user input --> 'pause' function; this command runs a loop in the background to have active response of the render window, e.g., to open the visualization dialog or use the right-mouse-button; behaves similar as SC.WaitForRenderEngineStopFlagthis()",
                        argList=['printMessage'],
                        defaultArgs=['True'],
                        returnType='None',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SendRedrawSignal', cName='SendRedrawSignal', 
                        description="this function is used to send a signal to the renderer that the scene shall be redrawn because the visualization state has been updated",
                        returnType='None',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetRenderEngineStopFlag', cName='GetRenderEngineStopFlag', 
                        description="get the current stop simulation flag; True=user wants to stop simulation",
                        returnType='bool',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetRenderEngineStopFlag', cName='SetRenderEngineStopFlag', 
                        description="set the current stop simulation flag; set to False, in order to continue a previously user-interrupted simulation",
                        argList=['stopFlag'],
                        argTypes=['bool'],
                        returnType='None',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='ActivateRendering', cName='ActivateRendering', 
                        description="activate (flag=True) or deactivate (flag=False) rendering for this system",
                        argList=['flag'],
                        argTypes=['bool'],
                        defaultArgs=['True'],
                        returnType='None',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetPreStepUserFunction', cName='PySetPreStepUserFunction', 
                        description="Sets a user function PreStepUserFunction(mbs, t) executed at beginning of every computation step; in normal case return True; return False to stop simulation after current step; set to 0 (integer) in order to erase user function. Note that the time returned is already the end of the step, which allows to compute forces consistently with trapezoidal integrators; for higher order Runge-Kutta methods, step time will be available only in object-user functions.",
                        example = 'def PreStepUserFunction(mbs, t):\\\\ \\TAB print(mbs.systemData.NumberOfNodes())\\\\ \\TAB if(t>1): \\\\ \\TAB  \\TAB return False \\\\ \\TAB return True \\\\mbs.SetPreStepUserFunction(PreStepUserFunction)',
                        argList=['value'],
                        argTypes=['Callable[[MainSystem, float],bool]'],
                        returnType='None',
                        )
                                                      
plr.DefPyFunctionAccess(cClass=classStr, pyName='GetPreStepUserFunction', cName='PyGetPreStepUserFunction', 
                        description="Returns the preStepUserFunction.",
                        argList=['asDict'],
                        argTypes=['bool'],
                        defaultArgs=['False'],
                        returnType='Callable[[MainSystem, float],bool]',
                        )
                                                      
plr.DefPyFunctionAccess(cClass=classStr, pyName='SetPostStepUserFunction', cName='PySetPostStepUserFunction', 
                        description="Sets a user function PostStepUserFunction(mbs, t) executed at beginning of every computation step; in normal case return True; return False to stop simulation after current step; set to 0 (integer) in order to erase user function.",
                        example = 'def PostStepUserFunction(mbs, t):\\\\ \\TAB print(mbs.systemData.NumberOfNodes())\\\\ \\TAB if(t>1): \\\\ \\TAB  \\TAB return False \\\\ \\TAB return True \\\\mbs.SetPostStepUserFunction(PostStepUserFunction)',
                        argList=['value'],
                        argTypes=['Callable[[MainSystem, float],bool]'],
                        returnType='None',
                        )
                                                      
plr.DefPyFunctionAccess(cClass=classStr, pyName='GetPostStepUserFunction', cName='PyGetPostStepUserFunction', 
                        description="Returns the postStepUserFunction.",
                        argList=['asDict'],
                        argTypes=['bool'],
                        defaultArgs=['False'],
                        returnType='Callable[[MainSystem, float],bool]',
                        )
                                                      
plr.DefPyFunctionAccess(cClass=classStr, pyName='SetPostNewtonUserFunction', cName='PySetPostNewtonUserFunction', 
                        description="Sets a user function PostNewtonUserFunction(mbs, t) executed after successful Newton iteration in implicit or static solvers and after step update of explicit solvers, but BEFORE PostNewton functions are called by the solver; function returns list [discontinuousError, recommendedStepSize], containing a error of the PostNewtonStep, which is compared to [solver].discontinuous.iterationTolerance. The recommendedStepSize shall be negative, if no recommendation is given, 0 in order to enforce minimum step size or a specific value to which the current step size will be reduced and the step will be repeated; use this function, e.g., to reduce step size after impact or change of data variables; set to 0 (integer) in order to erase user function. Similar described by Flores and Ambrosio, https://doi.org/10.1007/s11044-010-9209-8",
                        example = 'def PostNewtonUserFunction(mbs, t):\\\\ \\TAB if(t>1): \\\\ \\TAB  \\TAB return [0, 1e-6] \\\\ \\TAB return [0,0] \\\\mbs.SetPostNewtonUserFunction(PostNewtonUserFunction)',
                        argList=['value'],
                        argTypes=['Callable[[MainSystem, float],[float,float]]'],
                        returnType='None',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetPostNewtonUserFunction', cName='PyGetPostNewtonUserFunction', 
                        description="Returns the postNewtonUserFunction.",
                        argList=['asDict'],
                        argTypes=['bool'],
                        defaultArgs=['False'],
                        returnType='Callable[[MainSystem, float],bool]',
                        )
                                                      

#contact:                                      
plr.DefPyFunctionAccess(cClass=classStr, pyName='AddGeneralContact', cName='AddGeneralContact', 
                        description="add a new general contact, used to enable efficient contact computation between objects (nodes or markers)", 
                        options='py::return_value_policy::reference',
                        returnType='GeneralContact',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetGeneralContact', cName='GetGeneralContact', 
                        description="get read/write access to GeneralContact with index generalContactNumber stored in mbs; Examples shows how to access the GeneralContact object added with last AddGeneralContact() command:",
                        example = 'gc=mbs.GetGeneralContact(mbs.NumberOfGeneralContacts()-1)',
                        argList=['generalContactNumber'],
                        options='py::return_value_policy::reference',
                        argTypes=['int'],
                        returnType='GeneralContact',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='DeleteGeneralContact', cName='DeleteGeneralContact', 
                        description="delete GeneralContact with index generalContactNumber in mbs; other general contacts are resorted (index changes!)",
                        argList=['generalContactNumber'],
                        argTypes=['int'],
                        returnType='None',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='NumberOfGeneralContacts', cName='NumberOfGeneralContacts', 
                        description="Return number of GeneralContact objects in mbs", 
                        returnType='int',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetAvailableFactoryItems', cName='GetAvailableFactoryItems', 
                                description="get all available items to be added (nodes, objects, etc.); this is useful in particular in case of additional user elements to check if they are available; the available items are returned as dictionary, containing lists of strings for Node, Object, etc.",
                                returnType='dict',
                                )


#++++++++++++++++++++++++++++++++++++++++++++++++++
#see: https://pybind11.readthedocs.io/en/stable/upgrade.html
plr.DefPyFunctionAccess(cClass=classStr, pyName='GetDictionary', cName='GetDictionary', 
                        description="[UNDER DEVELOPMENT]: return the dictionary of the system data (todo: and state), e.g., to copy the system or for pickling",
                        argList=[],
                        argTypes=[],
                        returnType='dict',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetDictionary', cName='SetDictionary', 
                        description="[UNDER DEVELOPMENT]: set system data (todo: and state) from given dictionary; used for pickling",
                        argList=['systemDict'],
                        argTypes=['dict'],
                        returnType='None',
                        )

plr.sPy += pickleDictTemplateNew.replace('{ClassName}', classStr)
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

#old version, with variables: plr.DefPyFunctionAccess(cClass=classStr, pyName='__repr__', cName='[](const MainSystem &ms) {\n            return "<systemData: \\n" + ms.GetMainSystemData().PyInfoSummary() + "\\nmainSystem:\\n  variables = " + EXUstd::ToString(ms.variables) + "\\n  sys = " + EXUstd::ToString(ms.systemVariables) + "\\n>\\n"; }', 
plr.DefPyFunctionAccess(cClass=classStr, pyName='__repr__', cName='[](const MainSystem &ms) {\n            return "<systemData: \\n" + ms.GetMainSystemData().PyInfoSummary() + "\\nFor details see mbs.systemData, mbs.sys and mbs.variables\\n>\\n"; }', 
                        description="return the representation of the system, which can be, e.g., printed",
                        isLambdaFunction = True,
                        example = 'print(mbs)')

plr.sPy += '        .def_property("systemIsConsistent", &MainSystem::GetFlagSystemIsConsistent, &MainSystem::SetFlagSystemIsConsistent)\n' 
plr.DefLatexDataAccess('systemIsConsistent','this flag is used by solvers to decide, whether the system is in a solvable state; this flag is set to False as long as Assemble() has not been called; any modification to the system, such as Add...(), Modify...(), etc. will set the flag to False again; this flag can be modified (set to True), if a change of e.g.~an object (change of stiffness) or load (change of force) keeps the system consistent, but would normally lead to systemIsConsistent=False',
                       dataType='bool',
                       )

plr.sPy += '        .def_property("interactiveMode", &MainSystem::GetInteractiveMode, &MainSystem::SetInteractiveMode)\n' 
plr.DefLatexDataAccess('interactiveMode','set this flag to True in order to invoke a Assemble() command in every system modification, e.g. AddNode, AddObject, ModifyNode, ...; this helps that the system can be visualized in interactive mode.',
                       dataType='bool',
                       )

plr.sPy += '        .def_readwrite("variables", &MainSystem::variables, py::return_value_policy::reference)\n' 
plr.DefLatexDataAccess('variables','this dictionary may be used by the user to store model-specific data, in order to avoid global Python variables in complex models; mbs.variables["myvar"] = 42 ',
                       dataType='dict',
                       )

plr.sPy += '        .def_readwrite("sys", &MainSystem::systemVariables, py::return_value_policy::reference)\n' 
plr.DefLatexDataAccess('sys','this dictionary is used by exudyn Python libraries, e.g., solvers, to avoid global Python variables ',
                       dataType='dict',
                       )

plr.sPy += '        .def_property("solverSignalJacobianUpdate", &MainSystem::GetFlagSolverSignalJacobianUpdate, &MainSystem::SetFlagSolverSignalJacobianUpdate)\n' 
plr.DefLatexDataAccess('solverSignalJacobianUpdate','this flag is used by solvers to decide, whether the jacobian should be updated; at beginning of simulation and after jacobian computation, this flag is set automatically to False; use this flag to indicate system changes, e.g. during time integration  ',
                       dataType='bool',
                       )

plr.sPy += '        .def_readwrite("systemData", &MainSystem::mainSystemData, py::return_value_policy::reference)\n' 
plr.DefLatexDataAccess('systemData','Access to SystemData structure; enables access to number of nodes, objects, ... and to (current, initial, reference, ...) state variables (ODE2, AE, Data,...)',
                       dataType='SystemData',
                       )

plr.DefLatexFinishTable()#only finalize latex table

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#create extensions
plr.DefLatexStartClass('MainSystem extensions (create)',"This section represents extensions to MainSystem, which are direct calls to Python functions; the 'create' extensions to simplify the creation of multibody systems, such as CreateMassPoint(...); these extensions allow a more intuitive interaction with the MainSystem class, see the following example. For activation, import \\texttt{exudyn.mainSystemExtensions} or \\texttt{exudyn.utilities}", subSection=True,labelName='sec:mainsystem:pythonExtensionsCreate')

plr.AddDocuCodeBlock(code="""
import exudyn as exu           
from exudyn.utilities import * 
#alternative: import exudyn.mainSystemExtensions
SC = exu.SystemContainer()
mbs = SC.AddSystem()
#
#create rigid body
b1=mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000, sideLengths=[0.1,0.1,1]),
                       referencePosition = [1,0,0], 
                       gravity = [0,0,-9.81])
""")

plr.sLatex += '\\input{MainSystemCreateExt.tex}\n\n'

with open('generated/MainSystemCreateExt.rst', 'r') as f:
    plr.sRST += f.read()

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#function extensions
plr.DefLatexStartClass('MainSystem extensions (general)','This section represents general extensions to MainSystem, which are direct calls to Python functions, such as PlotSensor or SolveDynamic; these extensions allow a more intuitive interaction with the MainSystem class, see the following example. For activation, import \\texttt{exudyn.mainSystemExtensions} or \\texttt{exudyn.utilities}', subSection=True,labelName='sec:mainsystem:pythonExtensions')

plr.AddDocuCodeBlock(code="""
#this example sketches the usage 
#for complete examples see Examples/ or TestModels/ folders
#create some multibody system (mbs) first:
# ... 
#
#compute system degree of freedom: 
mbs.ComputeSystemDegreeOfFreedom(verbose=True)
#
#call solver function directly from mbs:
mbs.SolveDynamic(exu.SimulationSettings())
#
#plot sensor directly from mbs:
mbs.PlotSensor(...)
""")

plr.sLatex += '\\input{MainSystemExt.tex}\n\n'

with open('generated/MainSystemExt.rst', 'r') as f:
    plr.sRST += f.read()


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
                                example = "item = Rigid2D( referenceCoordinates= [1,0.5,0], initialVelocities= [10,0,0]) \\\\mbs.AddNode(item) \\\\" + "nodeDict = {'nodeType': 'Point', \\\\'referenceCoordinates': [1.0, 0.0, 0.0], \\\\'initialCoordinates': [0.0, 2.0, 0.0], \\\\'name': 'example node'} \\\\mbs.AddNode(nodeDict)",
                                argTypes=[itemDict],
                                returnType='NodeIndex',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetNodeNumber', cName='PyGetNodeNumber', 
                                description="get node's number by name (string)",
                                argList=['nodeName'],
                                example = "n = mbs.GetNodeNumber('example node')",
                                argTypes=['str'],
                                returnType='NodeIndex',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetNode', cName='PyGetNode',
                                description="get node's dictionary by node number (type NodeIndex)",
                                argList=['nodeNumber'],
                                example = "nodeDict = mbs.GetNode(0)",
                                argTypes=['NodeIndex'],
                                returnType='dict',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='ModifyNode', cName='PyModifyNode', 
                                description="modify node's dictionary by node number (type NodeIndex)",
                                argList=['nodeNumber','nodeDict'],
                                example = "mbs.ModifyNode(nodeNumber, nodeDict)",
                                argTypes=['NodeIndex', 'dict'],
                                returnType='None',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetNodeDefaults', cName='PyGetNodeDefaults', 
                                description="get node's default values for a certain nodeType as (dictionary)",
                                argList=['typeName'],
                                example = "nodeType = 'Point'\\\\nodeDict = mbs.GetNodeDefaults(nodeType)",
                                argTypes=['str'],
                                returnType='dict',
                                )

#plr.DefPyFunctionAccess(cClass=classStr, pyName='CallNodeFunction', cName='PyCallNodeFunction', 
#                                description="call specific node function",
#                                argList=['nodeNumber', 'functionName', 'args'],
#                                defaultArgs=['', '', 'py::dict()']
#                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetNodeOutput', cName='PyGetNodeOutputVariable', 
                                description="get the ouput of the node specified with the OutputVariableType; output may be scalar or array (e.g. displacement vector)",
                                argList=['nodeNumber','variableType','configuration'],
                                defaultArgs=['','','exu.ConfigurationType::Current'],
                                example = "mbs.GetNodeOutput(nodeNumber=0, variableType=exu.OutputVariableType.Displacement)",
                                argTypes=['NodeIndex','OutputVariableType','ConfigurationType'],
                                returnType=returnedArray,
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetNodeODE2Index', cName='PyGetNodeODE2Index', 
                                description="get index in the global ODE2 coordinate vector for the first node coordinate of the specified node",
                                argList=['nodeNumber'],
                                example = "mbs.GetNodeODE2Index(nodeNumber=0)",
                                argTypes=['NodeIndex'],
                                returnType='int',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetNodeODE1Index', cName='PyGetNodeODE1Index', 
                                description="get index in the global ODE1 coordinate vector for the first node coordinate of the specified node",
                                argList=['nodeNumber'],
                                example = "mbs.GetNodeODE1Index(nodeNumber=0)",
                                argTypes=['NodeIndex'],
                                returnType='int',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetNodeAEIndex', cName='PyGetNodeAEIndex', 
                                description="get index in the global AE coordinate vector for the first node coordinate of the specified node",
                                argList=['nodeNumber'],
                                example = "mbs.GetNodeAEIndex(nodeNumber=0)",
                                argTypes=['NodeIndex'],
                                returnType='int',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetNodeParameter', cName='PyGetNodeParameter', 
                                description="get nodes's parameter from node number (type NodeIndex) and parameterName; parameter names can be found for the specific items in the reference manual; for visualization parameters, use a 'V' as a prefix",
                                argList=['nodeNumber', 'parameterName'],
                                example = "mbs.GetNodeParameter(0, 'referenceCoordinates')",
                                argTypes=['NodeIndex','str'],
                                returnType='Any',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetNodeParameter', cName='PySetNodeParameter', 
                                description="set parameter 'parameterName' of node with node number (type NodeIndex) to value; parameter names can be found for the specific items in the reference manual; for visualization parameters, use a 'V' as a prefix",
                                argList=['nodeNumber', 'parameterName', 'value'],
                                example = "mbs.SetNodeParameter(0, 'Vshow', True)",
                                argTypes=['NodeIndex','str','Any'],
                                returnType='None',
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
                                example = "item = MassPoint(name='heavy object', nodeNumber=0, physicsMass=100) \\\\mbs.AddObject(item) \\\\" + "objectDict = {'objectType': 'MassPoint', \\\\'physicsMass': 10, \\\\'nodeNumber': 0, \\\\'name': 'example object'} \\\\mbs.AddObject(objectDict)",
                                argTypes=[itemDict],
                                returnType='ObjectIndex',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetObjectNumber', cName='PyGetObjectNumber', 
                                description="get object's number by name (string)",
                                argList=['objectName'],
                                example = "n = mbs.GetObjectNumber('heavy object')",
                                argTypes=['str'],
                                returnType='ObjectIndex',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetObject', cName='PyGetObject', 
                                description="get object's dictionary by object number (type ObjectIndex); NOTE: visualization parameters have a prefix 'V'; in order to also get graphicsData written, use addGraphicsData=True (which is by default False, as it would spoil the information)",
                                argList=['objectNumber','addGraphicsData'],
                                argTypes=['ObjectIndex','bool'],
                                defaultArgs=['','False'],
                                example = "objectDict = mbs.GetObject(0)",
                                returnType='dict',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='ModifyObject', cName='PyModifyObject', 
                                description="modify object's dictionary by object number (type ObjectIndex); NOTE: visualization parameters have a prefix 'V'",
                                argList=['objectNumber','objectDict'],
                                argTypes=['ObjectIndex','dict'],
                                example = "mbs.ModifyObject(objectNumber, objectDict)",
                                returnType='None',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetObjectDefaults', cName='PyGetObjectDefaults', 
                                description="get object's default values for a certain objectType as (dictionary)",
                                argList=['typeName'],
                                argTypes=['str'],
                                example = "objectType = 'MassPoint'\\\\objectDict = mbs.GetObjectDefaults(objectType)",
                                returnType='dict',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetObjectOutput', cName='PyGetObjectOutputVariable', 
                                description="get object's current output variable from object number (type ObjectIndex) and OutputVariableType; for connectors, it can only be computed for exu.ConfigurationType.Current configuration!",
                                argList=['objectNumber', 'variableType', 'configuration'],
                                argTypes=['ObjectIndex','OutputVariableType','ConfigurationType'],
                                defaultArgs=['','','exu.ConfigurationType::Current'],
                                returnType=returnedArray,
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetObjectOutputBody', cName='PyGetObjectOutputVariableBody', 
                                description="get body's output variable from object number (type ObjectIndex) and OutputVariableType, using the localPosition as defined in the body, and as used in MarkerBody and SensorBody",
                                argList=['objectNumber', 'variableType', 'localPosition', 'configuration'],
                                argTypes=['ObjectIndex','OutputVariableType',vector3D,'ConfigurationType'],
                                defaultArgs=['','','(std::vector<Real>)Vector3D({0,0,0})','exu.ConfigurationType::Current'],
                                example = "u = mbs.GetObjectOutputBody(objectNumber = 1, variableType = exu.OutputVariableType.Position, localPosition=[1,0,0], configuration = exu.ConfigurationType.Initial)",
                                returnType=returnedArray,
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetObjectOutputSuperElement', cName='PyGetObjectOutputVariableSuperElement', 
                                description="get output variable from mesh node number of object with type SuperElement (GenericODE2, FFRF, FFRFreduced - CMS) with specific OutputVariableType; the meshNodeNumber is the object's local node number, not the global node number!",
                                argList=['objectNumber', 'variableType', 'meshNodeNumber', 'configuration'],
                                argTypes=['ObjectIndex','OutputVariableType','int','ConfigurationType'],
                                defaultArgs=['','','','exu.ConfigurationType::Current'],
                                example = "u = mbs.GetObjectOutputSuperElement(objectNumber = 1, variableType = exu.OutputVariableType.Position, meshNodeNumber = 12, configuration = exu.ConfigurationType.Initial)",
                                returnType=returnedArray,
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetObjectParameter', cName='PyGetObjectParameter', 
                                description="get objects's parameter from object number (type ObjectIndex) and parameterName; parameter names can be found for the specific items in the reference manual; for visualization parameters, use a 'V' as a prefix; NOTE that BodyGraphicsData cannot be get or set, use dictionary access instead",
                                argList=['objectNumber', 'parameterName'],
                                argTypes=['ObjectIndex','str'],
                                example = "mbs.GetObjectParameter(objectNumber = 0, parameterName = 'nodeNumber')",
                                returnType='Any',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetObjectParameter', cName='PySetObjectParameter', 
                                description="set parameter 'parameterName' of object with object number (type ObjectIndex) to value;; parameter names can be found for the specific items in the reference manual; for visualization parameters, use a 'V' as a prefix; NOTE that BodyGraphicsData cannot be get or set, use dictionary access instead",
                                argList=['objectNumber', 'parameterName', 'value'],
                                argTypes=['ObjectIndex','str','Any'],
                                example = "mbs.SetObjectParameter(objectNumber = 0, parameterName = 'Vshow', value=True)",
                                returnType='None',
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
                                example = "item = MarkerNodePosition(name='my marker',nodeNumber=1) \\\\mbs.AddMarker(item)\\\\" + "markerDict = {'markerType': 'NodePosition', \\\\  'nodeNumber': 0, \\\\  'name': 'position0'}\\\\mbs.AddMarker(markerDict)",
                                argTypes=[itemDict],
                                returnType='MarkerIndex',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetMarkerNumber', cName='PyGetMarkerNumber', 
                                description="get marker's number by name (string)",
                                argList=['markerName'],
                                example = "n = mbs.GetMarkerNumber('my marker')",
                                argTypes=['str'],
                                returnType='MarkerIndex',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetMarker', cName='PyGetMarker', 
                                description="get marker's dictionary by index",
                                argList=['markerNumber'],
                                example = "markerDict = mbs.GetMarker(0)",
                                argTypes=['MarkerIndex'],
                                returnType='dict',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='ModifyMarker', cName='PyModifyMarker', 
                                description="modify marker's dictionary by index",
                                argList=['markerNumber','markerDict'],
                                example = "mbs.ModifyMarker(markerNumber, markerDict)",
                                argTypes=['MarkerIndex','dict'],
                                returnType='None',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetMarkerDefaults', cName='PyGetMarkerDefaults', 
                                description="get marker's default values for a certain markerType as (dictionary)",
                                argList=['typeName'],
                                example = "markerType = 'NodePosition'\\\\markerDict = mbs.GetMarkerDefaults(markerType)",
                                argTypes=['str'],
                                returnType='dict',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetMarkerParameter', cName='PyGetMarkerParameter', 
                                description="get markers's parameter from markerNumber and parameterName; parameter names can be found for the specific items in the reference manual",
                                argList=['markerNumber', 'parameterName'],
                                argTypes=['MarkerIndex','str'],
                                returnType='Any',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetMarkerParameter', cName='PySetMarkerParameter', 
                                description="set parameter 'parameterName' of marker with markerNumber to value; parameter names can be found for the specific items in the reference manual",
                                argList=['markerNumber', 'parameterName', 'value'],
                                argTypes=['MarkerIndex','str','Any'],
                                returnType='None',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetMarkerOutput', cName='PyGetMarkerOutputVariable', 
                                description="get the ouput of the marker specified with the OutputVariableType; currently only provides Displacement, Position and Velocity for position based markers, and RotationMatrix, Rotation and AngularVelocity(Local) for markers providing orientation; Coordinates and Coordinates_t available for coordinate markers",
                                argList=['markerNumber','variableType','configuration'],
                                defaultArgs=['','','exu.ConfigurationType::Current'],
                                example = "mbs.GetMarkerOutput(markerNumber=0, variableType=exu.OutputVariableType.Position)",
                                argTypes=['MarkerIndex','OutputVariableType','ConfigurationType'],
                                returnType=returnedArray,
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
                                example = "item = mbs.AddLoad(LoadForceVector(loadVector=[1,0,0], markerNumber=0, name='heavy load')) \\\\mbs.AddLoad(item)\\\\" + "loadDict = {'loadType': 'ForceVector',\\\\  'markerNumber': 0,\\\\  'loadVector': [1.0, 0.0, 0.0],\\\\  'name': 'heavy load'} \\\\mbs.AddLoad(loadDict)",
                                argTypes=[itemDict],
                                returnType='LoadIndex',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetLoadNumber', cName='PyGetLoadNumber', 
                                description="get load's number by name (string)",
                                argList=['loadName'],
                                example = "n = mbs.GetLoadNumber('heavy load')",
                                argTypes=['str'],
                                returnType='LoadIndex',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetLoad', cName='PyGetLoad', 
                                description="get load's dictionary by index",
                                argList=['loadNumber'],
                                example = "loadDict = mbs.GetLoad(0)",
                                argTypes=['LoadIndex'],
                                returnType='dict',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='ModifyLoad', cName='PyModifyLoad', 
                                description="modify load's dictionary by index",
                                argList=['loadNumber','loadDict'],
                                example = "mbs.ModifyLoad(loadNumber, loadDict)",
                                argTypes=['LoadIndex','dict'],
                                returnType='None',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetLoadDefaults', cName='PyGetLoadDefaults', 
                                description="get load's default values for a certain loadType as (dictionary)",
                                argList=['typeName'],
                                example = "loadType = 'ForceVector'\\\\loadDict = mbs.GetLoadDefaults(loadType)",
                                argTypes=['str'],
                                returnType='dict',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetLoadValues', cName='PyGetLoadValues', 
                                description="Get current load values, specifically if user-defined loads are used; can be scalar or vector-valued return value",
                                argList=['loadNumber'],
                                argTypes=['LoadIndex'],
                                returnType=returnedArray,
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetLoadParameter', cName='PyGetLoadParameter', 
                                description="get loads's parameter from loadNumber and parameterName; parameter names can be found for the specific items in the reference manual",
                                argList=['loadNumber', 'parameterName'],
                                argTypes=['LoadIndex','str'],
                                returnType='Any',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetLoadParameter', cName='PySetLoadParameter', 
                                description="set parameter 'parameterName' of load with loadNumber to value; parameter names can be found for the specific items in the reference manual",
                                argList=['loadNumber', 'parameterName', 'value'],
                                argTypes=['LoadIndex','str','Any'],
                                returnType='None',
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
                                example = "item = mbs.AddSensor(SensorNode(sensorType= exu.SensorType.Node, nodeNumber=0, name='test sensor')) \\\\mbs.AddSensor(item)\\\\" + "sensorDict = {'sensorType': 'Node',\\\\  'nodeNumber': 0,\\\\  'fileName': 'sensor.txt',\\\\  'name': 'test sensor'} \\\\mbs.AddSensor(sensorDict)",
                                argTypes=[itemDict],
                                returnType='SensorIndex',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetSensorNumber', cName='PyGetSensorNumber', 
                                description="get sensor's number by name (string)",
                                argList=['sensorName'],
                                example = "n = mbs.GetSensorNumber('test sensor')",
                                argTypes=['str'],
                                returnType='SensorIndex',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetSensor', cName='PyGetSensor', 
                                description="get sensor's dictionary by index",
                                argList=['sensorNumber'],
                                example = "sensorDict = mbs.GetSensor(0)",
                                argTypes=['SensorIndex'],
                                returnType='dict',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='ModifySensor', cName='PyModifySensor', 
                                description="modify sensor's dictionary by index",
                                argList=['sensorNumber','sensorDict'],
                                example = "mbs.ModifySensor(sensorNumber, sensorDict)",
                                argTypes=['SensorIndex','dict'],
                                returnType='None',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetSensorDefaults', cName='PyGetSensorDefaults', 
                                description="get sensor's default values for a certain sensorType as (dictionary)",
                                argList=['typeName'],
                                example = "sensorType = 'Node'\\\\sensorDict = mbs.GetSensorDefaults(sensorType)",
                                argTypes=['str'],
                                returnType='dict',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetSensorValues', cName='PyGetSensorValues', 
                                description="get sensors's values for configuration; can be a scalar or vector-valued return value!",
                                defaultArgs=['','exu.ConfigurationType::Current'],
                                argList=['sensorNumber', 'configuration'],
                                argTypes=['SensorIndex','ConfigurationType'],
                                returnType=returnedArray,
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetSensorStoredData', cName='PyGetSensorStoredData',
                                description="get sensors's internally stored data as matrix (all time points stored); rows are containing time and sensor values as obtained by sensor (e.g., time, and x, y, and z value of position)",
                                defaultArgs=[''],
                                argList=['sensorNumber'],
                                argTypes=['SensorIndex'],
                                returnType='ArrayLike',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetSensorParameter', cName='PyGetSensorParameter', 
                                description="get sensors's parameter from sensorNumber and parameterName; parameter names can be found for the specific items in the reference manual",
                                argList=['sensorNumber', 'parameterName'],
                                argTypes=['SensorIndex','str'],
                                returnType='Any',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetSensorParameter', cName='PySetSensorParameter', 
                                description="set parameter 'parameterName' of sensor with sensorNumber to value; parameter names can be found for the specific items in the reference manual",
                                argList=['sensorNumber', 'parameterName', 'value'],
                                argTypes=['SensorIndex','str','Any'],
                                returnType='None',
                                )

plr.DefLatexFinishTable() #Sensors

#now finalize pybind class, but do nothing on latex side (sL1 ignored)
plr2 = PyLatexRST()
plr2.DefPyFinishClass('MainSystem')
plr.sPy += plr2.PyStr() #only use Pybind string

savedPyi = plr.sPyi+savedPyi
plr.sPyi = ''


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
plr.CreateNewRSTfile('SystemData')
pyClassStr = 'SystemData'
classStr = 'Main'+pyClassStr
plr.DefPyStartClass(classStr,pyClassStr, '', labelName='sec:mbs:systemData',
                    forbidPythonConstructor=True)

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
#print detailed information on items:
mbs.systemData.Info()
#print LTG lists for objects and loads:
mbs.systemData.InfoLTG()
""")

plr.DefLatexStartTable(classStr)

plr.sPy += "\n//        General functions:\n"
#plr.sLatex += '\\\\ \n'+classStr+': General functions', 'These functions allow to obtain system information (e.g. for debug purposes)', subSection=True)

#+++++++++++++++++++++++++++++++++
#General functions:
plr.DefPyFunctionAccess(cClass=classStr, pyName='NumberOfLoads', cName='[](const MainSystemData& msd) {return msd.GetMainLoads().NumberOfItems(); }', 
                                description="return number of loads in system",
                                isLambdaFunction = True,
                                example = 'print(mbs.systemData.NumberOfLoads())',
                                returnType='int',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='NumberOfMarkers', cName='[](const MainSystemData& msd) {return msd.GetMainMarkers().NumberOfItems(); }', 
                                description="return number of markers in system",
                                isLambdaFunction = True,
                                example = 'print(mbs.systemData.NumberOfMarkers())',
                                returnType='int',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='NumberOfNodes', cName='[](const MainSystemData& msd) {return msd.GetMainNodes().NumberOfItems(); }', 
                                description="return number of nodes in system",
                                isLambdaFunction = True,
                                example = 'print(mbs.systemData.NumberOfNodes())',
                                returnType='int',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='NumberOfObjects', cName='[](const MainSystemData& msd) {return msd.GetMainObjects().NumberOfItems(); }', 
                                description="return number of objects in system",
                                isLambdaFunction = True,
                                example = 'print(mbs.systemData.NumberOfObjects())',
                                returnType='int',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='NumberOfSensors', cName='[](const MainSystemData& msd) {return msd.GetMainSensors().NumberOfItems(); }', 
                                description="return number of sensors in system",
                                isLambdaFunction = True,
                                example = 'print(mbs.systemData.NumberOfSensors())',
                                returnType='int',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='ODE2Size', cName='PyODE2Size', 
                                description="get size of ODE2 coordinate vector for given configuration (only works correctly after mbs.Assemble() )",
                                argList=['configurationType'],
                                argTypes=['ConfigurationType'],
                                defaultArgs=['exu.ConfigurationType::Current'], #exu will be removed for binding
                                example = "print('ODE2 size=',mbs.systemData.ODE2Size())",
                                returnType='int',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='ODE1Size', cName='PyODE1Size', 
                                description="get size of ODE1 coordinate vector for given configuration (only works correctly after mbs.Assemble() )",
                                argList=['configurationType'],
                                argTypes=['ConfigurationType'],
                                defaultArgs=['exu.ConfigurationType::Current'], #exu will be removed for binding
                                example = "print('ODE1 size=',mbs.systemData.ODE1Size())",
                                returnType='int',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='AEsize', cName='PyAEsize', 
                                description="get size of AE coordinate vector for given configuration (only works correctly after mbs.Assemble() )",
                                argList=['configurationType'],
                                argTypes=['ConfigurationType'],
                                defaultArgs=['exu.ConfigurationType::Current'], #exu will be removed for binding
                                example = "print('AE size=',mbs.systemData.AEsize())",
                                returnType='int',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='DataSize', cName='PyDataSize', 
                                description="get size of Data coordinate vector for given configuration (only works correctly after mbs.Assemble() )",
                                argList=['configurationType'],
                                argTypes=['ConfigurationType'],
                                defaultArgs=['exu.ConfigurationType::Current'], #exu will be removed for binding
                                example = "print('Data size=',mbs.systemData.DataSize())",
                                returnType='int',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SystemSize', cName='PySystemSize', 
                                description="get size of System coordinate vector for given configuration (only works correctly after mbs.Assemble() )",
                                argList=['configurationType'],
                                argTypes=['ConfigurationType'],
                                defaultArgs=['exu.ConfigurationType::Current'], #exu will be removed for binding
                                example = "print('System size=',mbs.systemData.SystemSize())",
                                returnType='int',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetTime', cName='PyGetStateTime', 
                                description="get configuration dependent time.",
                                argList=['configurationType'],
                                argTypes=['ConfigurationType'],
                                defaultArgs=['exu.ConfigurationType::Current'], #exu will be removed for binding
                                example = "mbs.systemData.GetTime(exu.ConfigurationType.Initial)",
                                returnType='float',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetTime', cName='PySetStateTime', 
                                description="set configuration dependent time; use this access with care, e.g. in user-defined solvers.",
                                argList=['newTime','configurationType'],
                                argTypes=['float','ConfigurationType'],
                                defaultArgs=['', 'exu.ConfigurationType::Current'], #exu will be removed for binding
                                example = "mbs.systemData.SetTime(10., exu.ConfigurationType.Initial)",
                                returnType='None',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='AddODE2LoadDependencies', cName='PyAddODE2LoadDependencies', 
                                description="advanced function for adding special dependencies of loads onto ODE2 coordinates, taking a list / numpy array of global ODE2 coordinates; this function needs to be called after Assemble() and needs to contain ODE2 coordinate indices; this list only affects implicit or static solvers if numericalDifferentiation.loadsJacobian=True; in this case, it may greatly improve convergence if loads with user functions depend on some system states, such as in a load with feedback control loop; the additional dependencies are not required, if doSystemWideDifferentiation=True, however the latter option being much less efficient. For more details, consider the file doublePendulum2DControl.py in the examples directory.",
                                argList=['loadNumber','globalODE2coordinates'],
                                defaultArgs=['',''],
                                example = "mbs.systemData.AddODE2LoadDependencies(0,[0,1,2])\\\\#add dependency of load 5 onto node 2 coordinates:\\\\nodeLTG2 = mbs.systemData.GetNodeLTGODE2(2)\\\\mbs.systemData.AddODE2LoadDependencies(5,nodeLTG2)",
                                argTypes=['float','List[int]'],
                                returnType='None',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='Info', cName='[](const MainSystemData& msd) {pout << msd.PyInfoDetailed(); }', 
                                description="print detailed information on every item; for short information use print(mbs)",
                                isLambdaFunction = True,
                                example = 'mbs.systemData.Info()',
                                returnType='None',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='InfoLTG', cName='[](const MainSystemData& msd) {pout << msd.PyInfoLTG(); }', 
                                description="print LTG information of objects and load dependencies",
                                isLambdaFunction = True,
                                example = 'mbs.systemData.InfoLTG()',
                                returnType='None',
                                )



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
                                argTypes=['ConfigurationType'],
                                defaultArgs=['exu.ConfigurationType::Current'],
                                example = "uCurrent = mbs.systemData.GetODE2Coordinates()",
                                returnType=returnedArray,
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetODE2Coordinates', cName='SetODE2Coords', 
                                description="set ODE2 system coordinates (displacements) for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!",
                                argList=['coordinates','configuration'],
                                argTypes=[listOrArray,'ConfigurationType'],
                                defaultArgs=['','exu.ConfigurationType::Current'],
                                example = "mbs.systemData.SetODE2Coordinates(uCurrent)",
                                returnType='None',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetODE2Coordinates_t', cName='GetODE2Coords_t', 
                                description="get ODE2 system coordinates (velocities) for given configuration (default: exu.Configuration.Current)",
                                argList=['configuration'],
                                argTypes=['ConfigurationType'],
                                defaultArgs=['exu.ConfigurationType::Current'],
                                example = "vCurrent = mbs.systemData.GetODE2Coordinates_t()",
                                returnType=returnedArray,
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetODE2Coordinates_t', cName='SetODE2Coords_t', 
                                description="set ODE2 system coordinates (velocities) for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!",
                                argList=['coordinates','configuration'],
                                argTypes=[listOrArray,'ConfigurationType'],
                                defaultArgs=['','exu.ConfigurationType::Current'],
                                example = "mbs.systemData.SetODE2Coordinates_t(vCurrent)",
                                returnType='None',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetODE2Coordinates_tt', cName='GetODE2Coords_tt', 
                                description="get ODE2 system coordinates (accelerations) for given configuration (default: exu.Configuration.Current)",
                                argList=['configuration'],
                                argTypes=['ConfigurationType'],
                                defaultArgs=['exu.ConfigurationType::Current'],
                                example = "vCurrent = mbs.systemData.GetODE2Coordinates_tt()",
                                returnType=returnedArray,
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetODE2Coordinates_tt', cName='SetODE2Coords_tt', 
                                description="set ODE2 system coordinates (accelerations) for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!",
                                argList=['coordinates','configuration'],
                                argTypes=[listOrArray,'ConfigurationType'],
                                defaultArgs=['','exu.ConfigurationType::Current'],
                                example = "mbs.systemData.SetODE2Coordinates_tt(aCurrent)",
                                returnType='None',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetODE1Coordinates', cName='GetODE1Coords', 
                                description="get ODE1 system coordinates (displacements) for given configuration (default: exu.Configuration.Current)",
                                argList=['configuration'],
                                argTypes=['ConfigurationType'],
                                defaultArgs=['exu.ConfigurationType::Current'],
                                example = "qCurrent = mbs.systemData.GetODE1Coordinates()",
                                returnType=returnedArray,
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetODE1Coordinates', cName='SetODE1Coords', 
                                description="set ODE1 system coordinates (velocities) for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!",
                                argList=['coordinates','configuration'],
                                argTypes=[listOrArray,'ConfigurationType'],
                                defaultArgs=['','exu.ConfigurationType::Current'],
                                example = "mbs.systemData.SetODE1Coordinates_t(qCurrent)",
                                returnType='None',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetODE1Coordinates_t', cName='GetODE1Coords_t', 
                                description="get ODE1 system coordinates (velocities) for given configuration (default: exu.Configuration.Current)",
                                argList=['configuration'],
                                argTypes=['ConfigurationType'],
                                defaultArgs=['exu.ConfigurationType::Current'],
                                example = "qCurrent = mbs.systemData.GetODE1Coordinates_t()",
                                returnType=returnedArray,
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetODE1Coordinates_t', cName='SetODE1Coords_t', 
                                description="set ODE1 system coordinates (displacements) for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!",
                                argList=['coordinates','configuration'],
                                argTypes=[listOrArray,'ConfigurationType'],
                                defaultArgs=['','exu.ConfigurationType::Current'],
                                example = "mbs.systemData.SetODE1Coordinates(qCurrent)",
                                returnType='None',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetAECoordinates', cName='GetAECoords', 
                                description="get algebraic equations (AE) system coordinates for given configuration (default: exu.Configuration.Current)",
                                argList=['configuration'],
                                argTypes=['ConfigurationType'],
                                defaultArgs=['exu.ConfigurationType::Current'],
                                example = "lambdaCurrent = mbs.systemData.GetAECoordinates()",
                                returnType=returnedArray,
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetAECoordinates', cName='SetAECoords', 
                                description="set algebraic equations (AE) system coordinates for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!",
                                argList=['coordinates','configuration'],
                                argTypes=[listOrArray,'ConfigurationType'],
                                defaultArgs=['','exu.ConfigurationType::Current'],
                                example = "mbs.systemData.SetAECoordinates(lambdaCurrent)",
                                returnType='None',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetDataCoordinates', cName='GetDataCoords', 
                                description="get system data coordinates for given configuration (default: exu.Configuration.Current)",
                                argList=['configuration'],
                                argTypes=['ConfigurationType'],
                                defaultArgs=['exu.ConfigurationType::Current'],
                                example = "dataCurrent = mbs.systemData.GetDataCoordinates()",
                                returnType=returnedArray,
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetDataCoordinates', cName='SetDataCoords', 
                                description="set system data coordinates for given configuration (default: exu.Configuration.Current); invalid vector size may lead to system crash!",
                                argList=['coordinates','configuration'],
                                argTypes=[listOrArray,'ConfigurationType'],
                                defaultArgs=['','exu.ConfigurationType::Current'],
                                example = "mbs.systemData.SetDataCoordinates(dataCurrent)",
                                returnType='None',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetSystemState', cName='PyGetSystemState', 
                                description="get system state for given configuration (default: exu.Configuration.Current); state vectors do not include the non-state derivatives ODE1_t and ODE2_tt and the time; function is copying data - not highly efficient; format of pyList: [ODE2Coords, ODE2Coords_t, ODE1Coords, AEcoords, dataCoords]",
                                argList=['configuration'],
                                argTypes=['ConfigurationType'],
                                defaultArgs=['exu.ConfigurationType::Current'], #exu will be removed for binding
                                example = "sysStateList = mbs.systemData.GetSystemState()",
                                returnType='List[List[float]]',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetSystemState', cName='PySetSystemState', 
                                description="set system data coordinates for given configuration (default: exu.Configuration.Current); invalid list of vectors / vector size may lead to system crash; write access to state vectors (but not the non-state derivatives ODE1_t and ODE2_tt and the time); function is copying data - not highly efficient; format of pyList: [ODE2Coords, ODE2Coords_t, ODE1Coords, AEcoords, dataCoords]",
                                argList=['systemStateList','configuration'],
                                argTypes=['List[List[float]]','ConfigurationType'],
                                defaultArgs=['','exu.ConfigurationType::Current'], #exu will be removed for binding
                                example = "mbs.systemData.SetSystemState(sysStateList, configuration = exu.ConfigurationType.Initial)",
                                returnType='None',
                                )


plr.DefLatexFinishTable()

#+++++++++++++++++++++++++++++++++
#LTG-functions:
plr.sPy += "\n//        LTG readout functions:\n"
plr.DefLatexStartClass(pyClassStr+': Get object LTG coordinate mappings', '', subSection=True, labelName='sec:systemData:ObjectLTG')

plr.AddDocu('This section provides access functions the \\ac{LTG}-lists for every object (body, constraint, ...) '+
            'in the system. For details on the \\ac{LTG} mapping, see \\refSection{sec:overview:ltgmapping}.')

plr.DefLatexStartTable(classStr+':object LTG coordinate mappings')

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetObjectLTGODE2', cName='PyGetObjectLocalToGlobalODE2', 
                                description="get object local-to-global coordinate mapping (list of global coordinate indices) for ODE2 coordinates; only available after Assemble()",
                                argList=['objectNumber'],
                                example = "ltgObject4 = mbs.systemData.GetObjectLTGODE2(4)",
                                argTypes=['int'],
                                returnType='List[int]',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetObjectLTGODE1', cName='PyGetObjectLocalToGlobalODE1', 
                                description="get object local-to-global coordinate mapping (list of global coordinate indices) for ODE1 coordinates; only available after Assemble()",
                                argList=['objectNumber'],
                                example = "ltgObject4 = mbs.systemData.GetObjectLTGODE1(4)",
                                argTypes=['int'],
                                returnType='List[int]',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetObjectLTGAE', cName='PyGetObjectLocalToGlobalAE', 
                                description="get object local-to-global coordinate mapping (list of global coordinate indices) for algebraic equations (AE) coordinates; only available after Assemble()",
                                argList=['objectNumber'],
                                example = "ltgObject4 = mbs.systemData.GetObjectLTGAE(4)",
                                argTypes=['int'],
                                returnType='List[int]',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetObjectLTGData', cName='PyGetObjectLocalToGlobalData', 
                                description="get object local-to-global coordinate mapping (list of global coordinate indices) for data coordinates; only available after Assemble()",
                                argList=['objectNumber'],
                                example = "ltgObject4 = mbs.systemData.GetObjectLTGData(4)",
                                argTypes=['int'],
                                returnType='List[int]',
                                )

#node LTG:
plr.DefPyFunctionAccess(cClass=classStr, pyName='GetNodeLTGODE2', cName='PyGetNodeLocalToGlobalODE2', 
                                description="get node local-to-global coordinate mapping (list of global coordinate indices) for ODE2 coordinates; only available after Assemble()",
                                argList=['nodeNumber'],
                                example = "ltgNode4 = mbs.systemData.GetNodeLTGODE2(4)",
                                argTypes=['int'],
                                returnType='List[int]',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetNodeLTGODE1', cName='PyGetNodeLocalToGlobalODE1', 
                                description="get node local-to-global coordinate mapping (list of global coordinate indices) for ODE1 coordinates; only available after Assemble()",
                                argList=['nodeNumber'],
                                example = "ltgNode4 = mbs.systemData.GetNodeLTGODE1(4)",
                                argTypes=['int'],
                                returnType='List[int]',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetNodeLTGAE', cName='PyGetNodeLocalToGlobalAE', 
                                description="get node local-to-global coordinate mapping (list of global coordinate indices) for AE coordinates; only available after Assemble()",
                                argList=['nodeNumber'],
                                example = "ltgNode4 = mbs.systemData.GetNodeLTGAE(4)",
                                argTypes=['int'],
                                returnType='List[int]',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetNodeLTGData', cName='PyGetNodeLocalToGlobalData', 
                                description="get node local-to-global coordinate mapping (list of global coordinate indices) for Data coordinates; only available after Assemble()",
                                argList=['nodeNumber'],
                                example = "ltgNode4 = mbs.systemData.GetNodeLTGData(4)",
                                argTypes=['int'],
                                returnType='List[int]',
                                )


plr.DefLatexFinishTable()

#now finalize pybind class, but do nothing on latex side (sL1 ignored)
plr2 = PyLatexRST()
plr2.DefPyFinishClass('SystemData')
plr.sPy += plr2.PyStr() #only use Pybind string

savedPyi = plr.sPyi+savedPyi
plr.sPyi = ''




#add symbolic as a submodule rather than a class (similar to exudyn, but in other file...)

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
plr.CreateNewRSTfile('Symbolic')

plr.AddDocu('The Symbolic sub-module in \\texttt{exudyn.symbolic} allows limited symbolic manipulations in \\codeName\\ and is currently under development ' + 
            'In particular, symbolic user functions can be created, which allow significant speedup of Python user functions. '+
            'However, \\mybold{always veryfy your symbolic expressions or user functions}, as behavior may be unexpected in some cases. ',
            section='Symbolic', sectionLevel=1,sectionLabel='sec:cinterface:symbolic')

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#currently, only latex + RST binding:
pyClassStr = 'symbolic.Real'
classStr = 'Symbolic::SReal'
sPyOld = plr.PyStr() #systemcontainer manually added in C++

plrsym = PyLatexRST('','', '') #plrsym: pyi part goes into separate symbolic.pyi file


plrsym.DefPyStartClass(classStr, pyClassStr, '', subSection=True)

plrsym.AddDocu("The symbolic Real type allows to replace Python's float by a symbolic quantity. "+
            'The \\texttt{symbolic.Real} may be directly set to a float and be evaluated as float. '+
            'However, turing on recording by using \texttt{exudyn.symbolic.SetRecording(True)} (on by default), '+
            'results are stored as expression trees, which may be evaluated in C++ or Python, '+
            'in particular in user functions, see the following example:'
            )

plrsym.AddDocuCodeBlock(code="""
import exudyn as exu
esym = exu.symbolic     #abbreviation
SymReal = esym.Real     #abbreviation

#create some variables
a = SymReal('a',42.)    #use named expression
b = SymReal(13)         #b is 13
c = a+b*7.+1.-3         #c stores expression tree
d = c                   #d and c are containing same tree!
print('a: ',a,' = ',a.Evaluate())
print('c: ',c,' = ',c.Evaluate())

#use special functions:
d = a+b*esym.sin(a)+esym.cos(SymReal(7))
print('d: ',d,' = ',d.Evaluate())

a.SetValue(14)          #variable a set to new value; influences d
print('d: ',d,' = ',d.Evaluate())

a = SymReal(1000)       #a is now a new variable; not updated in d!
print('d: ',d,' = ',d.Evaluate())

#compute derivatives (automatic differentiation):
x = SymReal("x",0.5)
f = a+b*esym.sin(x)+esym.cos(SymReal(7))+x**4
print('f=',f.Evaluate(), ', diff=',f.Diff(x))

#turn off recording of trees (globally for all symbolic.Real!):
esym.SetRecording(False)
x = SymReal(42) #now, only represents a value
y = x/3.       #directly evaluates to 14
""")

plrsym.AddDocu('To create a symbolic Real, use \\texttt{aa=symbolic.Real(1.23)} to build '+
            'a Python object aa with value 1.23. In order to use a named value, '+
            "use \\texttt{pi=symbolic.Real('pi',3.14)}. Note that in the following, "+
            "we use the abbreviation \\texttt{SymReal=exudyn.symbolic.Real}. "+
            "Member functions of \\texttt{SymReal}, which are \\mybold{not recorded}, are:")

plrsym.DefLatexStartTable(pyClassStr)

# def __init__(self, arg1: int, arg2: float) -> None: ...
plrsym.DefPyFunctionAccess(cClass=classStr, pyName='__init__', cName='', 
                        description="Construct symbolic.Real from float.",
                        argList=['value'],
                        argTypes=['float'],
                        returnType='None',
                        )

plrsym.DefPyFunctionAccess(cClass=classStr, pyName='__init__', cName='', 
                        description="Construct named symbolic.Real from name and float.",
                        argList=['name', 'value'],
                        argTypes=['str', 'float'],
                        returnType='None',
                        )

plrsym.DefPyFunctionAccess(cClass=classStr, pyName='SetValue', cName='', 
                        description="Set either internal float value or value of named expression; cannot change symbolic expressions.",
                        example = "b = SymReal(13)\\\\b.SetValue(14) #now b is 14\\\\#b.SetValue(a+3.) #not possible!",
                        argList=['valueInit'],
                        argTypes=['float'],
                        returnType='None',
                        )

plrsym.DefPyFunctionAccess(cClass=classStr, pyName='Evaluate', cName='', 
                        description="return evaluated expression (prioritized) or stored Real value.",
                        returnType='float',
                        )

plrsym.DefPyFunctionAccess(cClass=classStr, pyName='Diff', cName='', 
                        description="(UNTESTED!) return derivative of stored expression with respect to given symbolic named variable; NOTE: when defining the expression of the variable which shall be differentiated, the variable may only be changed with the SetValue(...) method hereafter!",
                        example = "x=SymReal('x',2)\\\\f=3*x+x**2*sin(x)\\\\f.Diff(x) #evaluate derivative w.r.t. x",
                        returnType='float',
                        argList=['var'],
                        argTypes=['symbolic.Real'],
                        )

plrsym.DefLatexDataAccess('value','access to internal float value, which is used in case that symbolic.Real has been built from a float (but without a name and without symbolic expression)',
                       dataType = 'float')

plrsym.DefLatexOperator('__float__','evaluation of expression and conversion of symbolic.Real to Python float',
                       returnType = 'float')

plrsym.DefLatexOperator('__str__','conversion of symbolic.Real to string',
                       returnType = 'str')

plrsym.DefLatexOperator('__repr__','representation of symbolic.Real in Python',
                       returnType = 'str')

plrsym.DefLatexFinishTable()#only finalize latex table


plrsym.AddDocu("The remaining operators and mathematical functions are recorded within expressions. "
            "Main mathematical operators for \\texttt{SymReal} exist, similar to "+
            "Python, such as:")

plrsym.AddDocuCodeBlock(code="""
a = SymReal(1)
b = SymReal(2)

r1 = a+b
r1 = a-b
r1 = a*b
r1 = a/b
r1 = -a
r1 = a**b

c = SymReal(3.3)
c += b
c -= b
c *= b
c /= b

c = (a == b)
c = (a != b)
c = (a < b)
c = (a > b)
c = (a <= b)
c = (a >= b)

#in most cases, we can also mix with float:
c = a*7 + SymReal.sin(8)
""")

plrsym.AddDocu("Mathematical functions may be called with an \\texttt{SymReal} or with a \\texttt{float}. "+
            "Most standard mathematical functions exist for \\texttt{symbolic}, e.g., as \\texttt{symbolic.abs}. "+
            "\\mybold{HINT}: function names are lower-case for compatibility with Python's math library. "+
            "Thus, you can easily exchange math.sin with esym.sin, and you may want to use a generic "+
            "name, such as myMath=symbolic in order to switch between Python and symbolic user functions. "+
            "The following functions exist:")

plrsym.sPyi += '\n#functions directly in symbolic module:\n'

pyClassStr = 'symbolic'
classStr = ''
#these functions are in symbolic.Real:
plrsym.DefLatexStartTable(pyClassStr)
fnList=['isfinite','abs',#'sign',
        'round','ceil','floor',
        'sqrt','exp','log',
        'sin','cos','tan','asin','acos','atan',
        'sinh','cosh','tanh','asinh','acosh','atanh',
                ]

for fnName in fnList:
    cfnName = fnName
    if fnName=='abs' or fnName=='mod':
        cfnName = 'f'+cfnName
    #empty class indicates that these are functions in symbolic:
    plrsym.DefPyFunctionAccess(cClass='', pyName=fnName, cName='', 
                            description="according to specification of C++ std::"+cfnName,
                            argList=['x'],
                            argTypes=['symbolic.Real'],
                            returnType='symbolic.Real',
                            )

plrsym.DefLatexFinishTable()#only finalize latex table

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++

plrsym.AddDocu("The following table lists special functions for \\texttt{SymReal}: ")

plrsym.DefLatexStartTable(pyClassStr)

plrsym.DefPyFunctionAccess(cClass=classStr, pyName='sign', cName='', 
                        description="returns 0 for x=0, -1 for x<0 and 1 for x>1.",
                        argList=['x'],
                        argTypes=['symbolic.Real'],
                        returnType='symbolic.Real',
                        )

plrsym.DefPyFunctionAccess(cClass=classStr, pyName='Not', cName='', 
                        description="returns logical not of expression, equal to Python's 'not'. Not(True)=False, Not(0.)=True, Not(-0.1)=False",
                        argList=['x'],
                        argTypes=['symbolic.Real'],
                        returnType='symbolic.Real',
                        )

plrsym.DefPyFunctionAccess(cClass=classStr, pyName='min', cName='', 
                        description="return minimum of x and y. ",
                        argList=['x','y'],
                        argTypes=['symbolic.Real','symbolic.Real'],
                        returnType='symbolic.Real',
                        )

plrsym.DefPyFunctionAccess(cClass=classStr, pyName='max', cName='', 
                        description="return maximum of x and y. ",
                        argList=['x','y'],
                        argTypes=['symbolic.Real','symbolic.Real'],
                        returnType='symbolic.Real',
                        )

plrsym.DefPyFunctionAccess(cClass=classStr, pyName='mod', cName='', 
                        description="return floating-point remainder of the division operation x / y. For example, mod(5.1, 3) gives 2.1 as a remainder.",
                        argList=['x','y'],
                        argTypes=['symbolic.Real','symbolic.Real'],
                        returnType='symbolic.Real',
                        )

plrsym.DefPyFunctionAccess(cClass=classStr, pyName='pow', cName='', 
                        description="return $x^y$. ",
                        argList=['x','y'],
                        argTypes=['symbolic.Real','symbolic.Real'],
                        returnType='symbolic.Real',
                        )

plrsym.DefPyFunctionAccess(cClass=classStr, pyName='max', cName='', 
                        description="return maximum of x and y. ",
                        argList=['x','y'],
                        argTypes=['symbolic.Real','symbolic.Real'],
                        returnType='symbolic.Real',
                        )

plrsym.DefPyFunctionAccess(cClass=classStr, pyName='IfThenElse', cName='', 
                        description="Symbolic function for conditional evaluation. If the condition evaluates to True, the expression ifTrue is evaluated, while otherwise expression ifFalse is evaluated",
                        example = "x=SymReal(-1)\\\\y=SymReal(2,'y')\\\\a=SymReal.IfThenElse(x<0, y+1, y-1))",
                        argList=['condition','ifTrue','ifFalse'],
                        argTypes=['symbolic.Real','symbolic.Real','symbolic.Real'],
                        returnType='symbolic.Real',
                        )

#+++++++++++++++++++++++
plrsym.DefPyFunctionAccess(cClass=classStr, pyName='SetRecording', cName='', 
                        description="Set current (global / module-wide) status of expression recording. By default, recording is on.",
                        example = "SymReal.SetRecording(True)",
                        argList=['flag'],
                        argTypes=['bool'],
                        returnType='None',
                        )

plrsym.DefPyFunctionAccess(cClass=classStr, pyName='GetRecording', cName='', 
                        description="Get current (global / module-wide) status of expression recording.",
                        example = "symbolic.Real.GetRecording()",
                        returnType='bool',
                        )

plrsym.DefLatexFinishTable()#only finalize latex table
plrsym.sPyi += '\n'

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#currently, only latex + RST binding:
pyClassStr = 'symbolic.Vector'
classStr = 'Symbolic::SymbolicRealVector'

plrsym.DefPyStartClass(classStr, pyClassStr, '', subSection=True)

plrsym.AddDocu("A symbolic Vector type to replace Python's (1D) numpy array in symbolic expressions. "+
            'The \\texttt{symbolic.Vector} may be directly set to a list of floats or (1D) numpy array and be evaluated as array. '+
            'However, turing on recording by using \texttt{exudyn.symbolic.SetRecording(True)} (on by default), '+
            'results are stored as expression trees, which may be evaluated in C++ or Python, '+
            'in particular in user functions, see the following example:'
            )

plrsym.AddDocuCodeBlock(code="""
import exudyn as exu
esym = exu.symbolic

SymVector = esym.Vector
SymReal = esym.Real 

a = SymReal('a',42.)
b = SymReal(13)
c = a-3*b

#create from list:
v1 = SymVector([1,3,2])
print('v1: ',v1)

#create from numpy array:
v2 = SymVector(np.array([1,3,2]))
print('v2 initial: ',v2)

#create from list, mixing symbolic expressions and numbers:
v2 = SymVector([a,42,c])

print('v2 now: ',v2,"=",v2.Evaluate())
print('v1+v2: ',v1+v2,"=",(v1+v2).Evaluate()) #evaluate as vector

print('v1*v2: ',v1*v2,"=",(v1*v2).Evaluate()) #evaluate as Real

#access of vector component:
print('v1[2]: ',v1[2],"=",v1[2].Evaluate())   #evaluate as Real
""")

plrsym.AddDocu('To create a symbolic Vector, use \\texttt{aa=symbolic.Vector([3,4.2,5]} to build '+
            'a Python object aa with values [3,4.2,5]. In order to use a named vector, '+
            "use \\texttt{v=symbolic.Vector('myVec',[3,4.2,5])}. "+
            "Vectors can be also created from mixed symbolic expressions and numbers, such as \\texttt{v=symbolic.Vector([x,x**2,3.14])}, "+
            "however, this cannot become a named vector as it contains expressions. "+
            "There is a significance difference to numpy, such that '*' represents the scalar vector multplication which gives a scalar. "+
            "Furthermore, the comparison operator '==' gives only True, if all components are equal, "+
            "and the operator '!=' gives True, if any component is unequal. "+
            "Note that in the following, we use the abbreviation \\texttt{SymVector=exudyn.symbolic.Vector}. "+
            "Note that only functions are able to be recorded. "
            "Member functions of \\texttt{SymVector} are:")

plrsym.DefLatexStartTable(pyClassStr)

plrsym.DefPyFunctionAccess(cClass=classStr, pyName='__init__', cName='', 
                        description="Construct symbolic.Vector from vector represented as numpy array or list (which may contain symbolic expressions).",
                        argList=['vector'],
                        argTypes=['List[float]'],
                        returnType='None',
                        )

plrsym.DefPyFunctionAccess(cClass=classStr, pyName='__init__', cName='', 
                        description="Construct named symbolic.Vector from name and vector represented as numpy array or list (which may contain symbolic expressions).",
                        argList=['name', 'vector'],
                        argTypes=['str', 'List[float]'],
                        returnType='None',
                        )

plrsym.DefPyFunctionAccess(cClass=classStr, pyName='Evaluate', cName='', 
                        description="Return evaluated expression (prioritized) or stored vector value. (not recorded)",
                        returnType='List[float]',
                        )

plrsym.DefPyFunctionAccess(cClass=classStr, pyName='SetVector', cName='', 
                        description="Set stored vector or named vector expression to new given (non-symbolic) vector. Only works, if SymVector contains no expression. (may lead to inconsistencies in recording)",
                        argList=['vector'],
                        argTypes=['symbolic.Vector'],
                        returnType='None',
                        )

plrsym.DefPyFunctionAccess(cClass=classStr, pyName='NumberOfItems', cName='', 
                        description="Get size of Vector (may require to evaluate expression; not recording)",
                        returnType='int',
                        )

# plrsym.DefPyFunctionAccess(cClass=classStr, pyName='__setitem__', cName='', 
#                         description="bracket [] operator for setting a component of the vector. Only works, if SymVector contains no expression. (may lead to inconsistencies in recording)",
#                         example = "v1 = SymVector([1,3,2])\\\\v1[2]=13.",
#                         argList=['i'],
#                         argTypes=['symbolic.Real'],
#                         returnType='None',
#                         )
plrsym.DefLatexOperator(name='__setitem__', 
                     description="bracket [] operator for setting a component of the vector. Only works, if SymVector contains no expression. (may lead to inconsistencies in recording)",
                     argList=['index'],
                     argTypes=['symbolic.Real'],
                     returnType='symbolic.Real',
                     )

#recorded:
plrsym.DefPyFunctionAccess(cClass=classStr, pyName='NormL2', cName='', 
                        description="return (symbolic) L2-norm of vector.",
                        example = "v1 = SymVector([1,4,8])\\\\length = v1.NormL2() #gives 9.",
                        returnType='symbolic.Real',
                        )

plrsym.DefPyFunctionAccess(cClass=classStr, pyName='MultComponents', cName='', 
                        description="Perform component-wise multiplication of vector times other vector and return result. This corresponds to the numpy multiplication using '*'.",
                        example = "v1 = SymVector([1,2,4])\\\\v2 = SymVector([1,0.5,0.25])\\\\v3 = v1.MultComponents(v2)",
                        argList=['other'],
                        argTypes=['symbolic.Vector'],
                        returnType='symbolic.Real',
                        )

plrsym.DefLatexOperator(name='__getitem__', 
                     description="bracket [] operator to return (symbolic) component of vector, allowing read-access. Index may also evaluate from an expression.",
                     argList=['index'],
                     argTypes=['symbolic.Real'],
                     returnType='symbolic.Real',
                     )


plrsym.DefLatexOperator('__str__','conversion of SymVector to string',
                       returnType = 'str')

plrsym.DefLatexOperator('__repr__','representation of SymVector in Python',
                       returnType = 'str')


#+++++++++
plrsym.DefLatexFinishTable()#only finalize latex table
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++

plrsym.AddDocu("Standard vector operators are available for \\texttt{SymVector}, see the following examples:")

plrsym.AddDocuCodeBlock(code="""
v = SymVector([1,3,2])
w = SymVector([3.3,2.2,1.1])

u = v+w
u = v-w
u = -v
#scalar multiplication; evaluates to SymReal:
x = v*w 
#NOTE: component-wise multiplication, returns SymVector:
u = v.MultComponents(w)

#inplace operators:
v += w
v -= w
v *= SymReal(0.5)
""")

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#currently, only latex + RST binding:
pyClassStr = 'symbolic.Matrix'
classStr = 'Symbolic::SymbolicRealMatrix'

plrsym.DefPyStartClass(classStr, pyClassStr, '', subSection=True)

plrsym.AddDocu("A symbolic Matrix type to replace Python's (2D) numpy array in symbolic expressions. "+
            'The \\texttt{symbolic.Matrix} may be directly set to a list of list of floats or (2D) numpy array and be evaluated as array. '+
            'However, turing on recording by using \texttt{exudyn.symbolic.SetRecording(True)} (on by default), '+
            'results are stored as expression trees, which may be evaluated in C++ or Python, '+
            'in particular in user functions, see the following example:'
            )

plrsym.AddDocuCodeBlock(code="""
import exudyn as exu
import numpy as np
esym = exu.symbolic

SymMatrix = esym.Matrix
SymReal = esym.Real 

a = SymReal('a',42.)
b = SymReal(13)

#create matrix from list of lists
m1 = SymMatrix([[1,3,2],[4,5,6]])

#create symbolic matrix from list of lists
m3 = SymMatrix([[a,3*b,2],[4,5,6]])

#create from numpy array
m2 = SymMatrix(np.ones((3,3))-np.eye(3))

m1 += m3
m1 *= 3
m1 -= 3*m3
print('m1: ',m1)
print('m2: ',m2)

""")

plrsym.AddDocu('To create a symbolic Matrix, use \\texttt{aa=symbolic.Matrix([[3,4.2],[3.3,1.2]]} to build '+
            'a Python object aa. In order to use a named matrix, '+
            "use \\texttt{v=symbolic.Matrix('myMat',[3,4.2,5])}. "+
            "Matrixs can be also created from mixed symbolic expressions and numbers, such as \\texttt{v=symbolic.Matrix([x,x**2,3.14])}, "+
            "however, this cannot become a named matrix as it contains expressions. "+
            "There is a significance difference to numpy, such that '*' represents the matrix multplication (compute components from row times column operations). "+
            "Note that in the following, we use the abbreviation \\texttt{SymMatrix=exudyn.symbolic.Matrix}. "+
            "Member functions of \\texttt{SymMatrix} are:")

plrsym.DefLatexStartTable(pyClassStr)

plrsym.DefPyFunctionAccess(cClass=classStr, pyName='__init__', cName='', 
                        description="Construct symbolic.Matrix from vector represented as numpy array or list of lists (which may contain symbolic expressions).",
                        argList=['matrix'],
                        argTypes=['List[List[float]]'],
                        returnType='None',
                        )

plrsym.DefPyFunctionAccess(cClass=classStr, pyName='__init__', cName='', 
                        description="Construct named symbolic.Matrix from name and vector represented as numpy array or list of lists (which may contain symbolic expressions).",
                        argList=['name', 'matrix'],
                        argTypes=['str', 'List[List[float]]'],
                        returnType='None',
                        )

plrsym.DefPyFunctionAccess(cClass=classStr, pyName='Evaluate', cName='', 
                        description="Return evaluated expression (prioritized) or stored Matrix value. (not recorded)",
                        returnType='List[float]',
                        )

plrsym.DefPyFunctionAccess(cClass=classStr, pyName='SetMatrix', cName='', 
                        description="Set stored Matrix or named Matrix expression to new given (non-symbolic) Matrix. Only works, if SymMatrix contains no expression. (may lead to inconsistencies in recording)",
                        argList=['matrix'],
                        argTypes=['NDArray[Any, float]'],
                        returnType='None',
                        )

plrsym.DefPyFunctionAccess(cClass=classStr, pyName='NumberOfRows', cName='', 
                        description="Get number of rows (may require to evaluate expression; not recording)",
                        returnType='int',
                        )

plrsym.DefPyFunctionAccess(cClass=classStr, pyName='NumberOfColumns', cName='', 
                        description="Get number of columns (may require to evaluate expression; not recording)",
                        returnType='int',
                        )

plrsym.DefLatexOperator(name='__setitem__', 
                     description="bracket [] operator for (symbolic) component of Matrix (write-access). Only works, if SymMatrix contains no expression. (may lead to inconsistencies in recording)",
                     argList=['row','column'],
                     argTypes=['symbolic.Real','symbolic.Real'],
                     returnType='symbolic.Real',
                     )

#recorded:
# plrsym.DefPyFunctionAccess(cClass=classStr, pyName='NormL2', cName='', 
#                         description="return (symbolic) L2-norm of Matrix.",
#                         example = "v1 = SymMatrix([1,4,8])\\\\length = v1.NormL2() #gives 9.",
#                         returnType='symbolic.Real',
#                         )

# plrsym.DefPyFunctionAccess(cClass=classStr, pyName='MultComponents', cName='', 
#                         description="Perform component-wise multiplication of Matrix times other Matrix and return result. This corresponds to the numpy multiplication using '*'.",
#                         example = "v1 = SymMatrix([1,2,4])\\\\v2 = SymMatrix([1,0.5,0.25])\\\\v3 = v1.MultComponents(v2)",
#                         argList=['other'],
#                         argTypes=['sym.Matrix'],
#                         returnType='symbolic.Real',
#                         )

plrsym.DefLatexOperator(name='__getitem__', 
                     description="bracket [] operator for (symbolic) component of Matrix (read-access). Row and column may also evaluate from an expression.",
                     argList=['row','column'],
                     argTypes=['symbolic.Real','symbolic.Real'],
                     returnType='symbolic.Real',
                     )


plrsym.DefLatexOperator('__str__','conversion of SymMatrix to string',
                     returnType = 'str')

plrsym.DefLatexOperator('__repr__','representation of SymMatrix in Python',
                     returnType = 'str')


#+++++++++
plrsym.DefLatexFinishTable()#only finalize latex table
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++

plrsym.AddDocu("Standard Matrix operators are available for \\texttt{SymMatrix}, see the following examples:")

plrsym.AddDocuCodeBlock(code="""
m1 = SymMatrix([[1,7],[4,5]])
m2 = SymMatrix([[1,2.2],[4,4.3]])
v = SymVector([1.5,3])

m3 = m1+m2
m3 = m1-m2
m3 = m1*m2

#multiply with scalar
m3 = 13*m2
m3 = m2*3.14

#multiply with vector
m3 = m2*v

#transposed:
m3 = v*m2 #equals numpy operation m2.T @ v

#inplace operators:
m1 += m1
m1 -= m1
m1 *= 3.14
""")


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#currently, only latex + RST binding:
pyClassStr = 'symbolic.VariableSet'
classStr = 'Symbolic::VariableSet'

plrsym.DefPyStartClass(classStr, pyClassStr, '', subSection=True)

plrsym.AddDocu("A container for symbolic variables, in particular for exchange between "+
            "user functions and the model. "+
            "For details, see the following example:"
            )

plrsym.AddDocuCodeBlock(code="""
import exudyn as exu
import math
SymReal = exu.symbolic.Real

#use global variable set:
variables = exu.symbolic.variables

#create a named Real
a = SymReal('a',42.)

#regular way to add variable:
variables.Add('pi', math.pi)

#add named variable (doesn't need a name):
variables.Add(a)

#print current variable set
print(variables)

print('pi=',variables.Get('pi').Evaluate()) #3.14
print('a=',variables.Get('a')) #prints 'a'

x=variables.Get('a')
print('x=',x.Evaluate()) #x=42

#override a
variables.Set('a',3.33)

#x is depending on a:
print('x:',x,"=",x.Evaluate()) #3.33

#create your own variable set
mySet = esym.VariableSet()
""")

plrsym.DefLatexStartTable(pyClassStr)

# 		//+++++++++++++++++++++++++++++++++++++++++++

# 		.def("__getitem__", [](Symbolic::SymbolicVariableSet& self, std::string name)
# 			{ return self.GetVariable(name); })
# 		.def("__setitem__", [](Symbolic::SymbolicVariableSet& self, std::string name, Real value)
# 			{ return self.SetVariable(name, value); })

plrsym.DefPyFunctionAccess(cClass=classStr, pyName='Add', cName='', 
                        description="Add a variable with name and value (name may not exist)",
                        argList=['name','value'],
                        argTypes=['str','float'],
                        returnType='None',
                        )

plrsym.DefPyFunctionAccess(cClass=classStr, pyName='Add', cName='', 
                        description="Add a variable with named real (name may not exist)",
                        argList=['namedReal'],
                        argTypes=['symbolic.Real'],
                        returnType='None',
                        )

plrsym.DefPyFunctionAccess(cClass=classStr, pyName='Set', cName='', 
                        description="Set a variable with name and value (adds new or overrides existing)",
                        argList=['name','value'],
                        argTypes=['str','float'],
                        returnType='None',
                        )

plrsym.DefPyFunctionAccess(cClass=classStr, pyName='Get', cName='', 
                        description="Get a variable by name",
                        argList=['name'],
                        argTypes=['str'],
                        returnType='symbolic.Real',
                        )

plrsym.DefPyFunctionAccess(cClass=classStr, pyName='Exists', cName='', 
                        description="Return True, if variable name exists",
                        argList=['name'],
                        argTypes=['str'],
                        returnType='bool',
                        )

plrsym.DefPyFunctionAccess(cClass=classStr, pyName='Reset', cName='', 
                        description="Erase all variables and reset VariableSet",
                        returnType='None',
                        )

plrsym.DefPyFunctionAccess(cClass=classStr, pyName='NumberOfItems', cName='', 
                        description="Return True, if variable name exists",
                        argList=['name'],
                        argTypes=['str'],
                        returnType='bool',
                        )

plrsym.DefPyFunctionAccess(cClass=classStr, pyName='GetNames', cName='', 
                        description="Get list of stored variable names",
                        returnType='List[str]',
                        )

plrsym.DefPyFunctionAccess(cClass=classStr, pyName='__setitem__', cName='', 
                        description="bracket [] operator for setting a variable to a specific value",
                        argList=['name','value'],
                        argTypes=['str','float'],
                        returnType='None',
                        )

plrsym.DefPyFunctionAccess(cClass=classStr, pyName='__getitem__', cName='', 
                        description="bracket [] operator for getting a specific variable by name",
                        argList=['name'],
                        argTypes=['str'],
                        returnType='symbolic.Real',
                        )


plrsym.DefLatexOperator('__str__','create string of set of variables',
                       returnType = 'str')

plrsym.DefLatexOperator('__repr__','representation of SymMatrix in Python',
                       returnType = 'str')


#+++++++++
plrsym.DefLatexFinishTable()#only finalize latex table
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#currently, only latex + RST binding:
pyClassStr = 'symbolic.UserFunction'
classStr = 'Symbolic::PySymbolicUserFunction'

plrsym.DefPyStartClass(classStr, pyClassStr, '', subSection=True)

plrsym.AddDocu("A class for creating and handling symbolic user functions in C++. "+
            "Use these functions for high performance extensions, e.g., of existing objects or loads"+
            "For details, see the following example:"
            )

plrsym.AddDocuCodeBlock(code="""
import exudyn as exu
esym = exu.symbolic
from exudyn.utilities import * #advancedUtilities with user function utilities included
SymReal = exu.symbolic.Real

SC = exu.SystemContainer()
mbs = SC.AddSystem()

#regular Python user function with esym math functions
def UFload(mbs, t, load):
    return load*esym.sin(10*(2*pi)*t)

#create symbolic user function from Python user function:
symFuncLoad = CreateSymbolicUserFunction(mbs, UFload, load, 'loadUserFunction',verbose=1)

#add ground and mass point:
oGround = mbs.CreateGround()
oMassPoint = mbs.CreateMassPoint(referencePosition=[1.+0.05,0,0], physicsMass=1)

#add marker and load:
mc = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=mbs.GetObject(oMassPoint)['nodeNumber'], coordinate=0))
load = mbs.AddLoad(LoadCoordinate(markerNumber=mc, load=10,
                                  loadUserFunction=symFuncLoad))

#print string of symbolic expression of user function (to check if it looks ok):
print('load user function: ',symFuncLoad)

#test evaluate user function; requires args of user function:
print('load user function: ',symFuncLoad.Evaluate(mbs, 0.025, 10.))
    
#now you could add further items or simulate ...
""")

plrsym.DefLatexStartTable(pyClassStr)

plrsym.DefPyFunctionAccess(cClass=classStr, pyName='Evaluate', cName='', 
                        description="Evaluate symbolic function with test values; requires exactly same args as Python user functions; this is slow and only intended for testing",
                        # returnType='None', #Real or Vector
                        )

plrsym.DefPyFunctionAccess(cClass=classStr, pyName='SetUserFunctionFromDict', cName='', 
                        description="Create C++ std::function (as requested in C++ item) with symbolic user function as recorded in given dictionary, as created with ConvertFunctionToSymbolic(...).",
                        argList=['mainSystem','fcnDict','itemIndex','userFunctionName'],
                        argTypes=['MainSystem','dict','ItemIndex','str'],
                        returnType='None',
                        )

# not needed any more (use userFunction directly, same as Python function)
# plrsym.DefPyFunctionAccess(cClass=classStr, pyName='TransferUserFunction2Item', cName='', 
#                         description="Transfer the std::function to a given object, load or other; this needs to be done purely in C++ to avoid Pybind overheads.",
#                         argList=['mainSystem','itemIndex','userFunctionName'],
#                         argTypes=['MainSystem','ItemIndex','str'],
#                         returnType='None',
#                         )

plrsym.DefLatexOperator('__repr__','Representation of Symbolic function',
                       returnType = 'str')

plrsym.DefLatexOperator('__str__','Convert stored symbolic function to string',
                       returnType = 'str')


#+++++++++
plrsym.DefLatexFinishTable()#only finalize latex table
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++

#plrsym.sPyi = plrsym.sPyi.replace('class symbolic.','class ')
plrsym.sPyi = plrsym.sPyi.replace('symbolic.','')
#plrsym.sPy = sPyOld  #symbolic added manually in .cpp

#add RST and Latex parts to plr
plr.sRST += plrsym.sRST
plr.sLatex += plrsym.sLatex
#savedPyi = plr.sPyi+savedPyi
#plr.sPyi = ''


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++













#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
plr.CreateNewRSTfile('GeneralContact')
classStr = 'PyGeneralContact'
pyClassStr = 'GeneralContact'
plr.DefPyStartClass(classStr, pyClassStr, '',labelName='sec:GeneralContact',
                    forbidPythonConstructor=True)

plr.AddDocu('Structure to define general and highly efficient contact functionality in multibody systems'+
            '\\footnote{Note that GeneralContact is still developed, use with care.}. For further explanations '+
            'and theoretical backgrounds, see \\refSection{secContactTheory}. '+
            'Internally, the contacts are stored with global indices, which are in the following list: '+
            '[numberOfSpheresMarkerBased, numberOfANCFCable2D, numberOfTrigsRigidBodyBased], see also'+
            'the output of GetPythonObject().')

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
                                description="convert member variables of GeneralContact into dictionary; use this for debug only!",
                                returnType='dict',
                                )

plr.DefPyFunctionAccess(cClass=classStr, pyName='Reset', cName='Reset', 
                                argList=['freeMemory'],
                                defaultArgs=['True'],
                                description="remove all contact objects and reset contact parameters",
                                returnType='None',
                                )

plr.sPy +=  '        .def_readwrite("isActive", &PyGeneralContact::isActive, py::return_value_policy::reference)\n' 
plr.DefLatexDataAccess('isActive','default = True (compute contact); if isActive=False, no contact computation is performed for this contact set ',
                       dataType='bool',
                       )

plr.sPy +=  '        .def_readwrite("verboseMode", &PyGeneralContact::verboseMode, py::return_value_policy::reference)\n' 
plr.DefLatexDataAccess('verboseMode','default = 0; verboseMode = 1 or higher outputs useful information on the contact creation and computation ',
                       dataType='int',
                       )

plr.sPy +=  '        .def_readwrite("visualization", &PyGeneralContact::visualization, py::return_value_policy::reference)\n' 
plr.DefLatexDataAccess('visualization','access visualization data structure ',
                       dataType='VisuGeneralContact',
                       )

plr.sPy +=  '        .def_property("resetSearchTreeInterval", &PyGeneralContact::GetResetSearchTreeInterval, &PyGeneralContact::SetResetSearchTreeInterval)\n' 
plr.DefLatexDataAccess('resetSearchTreeInterval','(default=10000) number of search tree updates (contact computation steps) after which the search tree cells are re-created; this costs some time, will free memory in cells that are not needed any more ',
                       dataType='int',
                       )

plr.sPy +=  '        .def_property("sphereSphereContact", &PyGeneralContact::GetSphereSphereContact, &PyGeneralContact::SetSphereSphereContact)\n' 
plr.DefLatexDataAccess('sphereSphereContact','activate/deactivate contact between spheres ',
                       dataType='bool',
                       )

plr.sPy +=  '        .def_property("sphereSphereFrictionRecycle", &PyGeneralContact::GetSphereSphereFrictionRecycle, &PyGeneralContact::SetSphereSphereFrictionRecycle)\n' 
plr.DefLatexDataAccess('sphereSphereFrictionRecycle','False: compute static friction force based on tangential velocity; True: recycle friction from previous PostNewton step, which greatly improves convergence, but may lead to unphysical artifacts; will be solved in future by step reduction ',
                       dataType='bool',
                       )

plr.sPy +=  '        .def_property("minRelDistanceSpheresTriangles", &PyGeneralContact::GetMinRelDistanceSpheresTriangles, &PyGeneralContact::SetMinRelDistanceSpheresTriangles)\n' 
plr.DefLatexDataAccess('minRelDistanceSpheresTriangles','(default=1e-10) tolerance (relative to sphere radiues) below which the contact between triangles and spheres is ignored; used for spheres directly attached to triangles ',
                       dataType='float',
                       )

plr.sPy +=  '        .def_property("frictionProportionalZone", &PyGeneralContact::GetFrictionProportionalZone, &PyGeneralContact::SetFrictionProportionalZone)\n' 
plr.DefLatexDataAccess('frictionProportionalZone','(default=0.001) velocity $v_{\mu,reg}$ upon which the dry friction coefficient is interpolated linearly (regularized friction model); must be greater 0; very small values cause oscillations in friction force ',
                       dataType='float',
                       )

plr.sPy +=  '        .def_property("frictionVelocityPenalty", &PyGeneralContact::GetFrictionVelocityPenalty, &PyGeneralContact::SetFrictionVelocityPenalty)\n' 
plr.DefLatexDataAccess('frictionVelocityPenalty','(default=1e3) regularization factor for friction [N/(m$^2 \cdot$m/s) ];$k_{\mu,reg}$, multiplied with tangential velocity to compute friciton force as long as it is smaller than $\mu$ times contact force; large values cause oscillations in friction force ',
                       dataType='float',
                       )

plr.sPy +=  '        .def_property("excludeOverlappingTrigSphereContacts", &PyGeneralContact::GetExcludeOverlappingTrigSphereContacts, &PyGeneralContact::SetExcludeOverlappingTrigSphereContacts)\n' 
plr.DefLatexDataAccess('excludeOverlappingTrigSphereContacts','(default=True) for consistent, closed meshes, we can exclude overlapping contact triangles (which would cause holes if mesh is overlapping and not consistent!!!) ',
                       dataType='bool',
                       )

plr.sPy +=  '        .def_property("excludeDuplicatedTrigSphereContactPoints", &PyGeneralContact::GetExcludeDuplicatedTrigSphereContactPoints, &PyGeneralContact::SetExcludeDuplicatedTrigSphereContactPoints)\n' 
plr.DefLatexDataAccess('excludeDuplicatedTrigSphereContactPoints','(default=False) run additional checks for double contacts at edges or vertices, being more accurate but can cause additional costs if many contacts ',
                       dataType='bool',
                       )

plr.sPy +=  '        .def_property("computeContactForces", &PyGeneralContact::GetComputeContactForces, &PyGeneralContact::SetComputeContactForces)\n' 
plr.DefLatexDataAccess('computeContactForces','(default=False) if True, additional system vector is computed which contains all contact force and torque contributions. In order to recover forces on a single rigid body, the respective LTG-vector has to be used and forces need to be extracted from this system vector; may slow down computations.',
                       dataType='bool',
                       )

plr.sPy +=  '        .def_property("ancfCableUseExactMethod", &PyGeneralContact::GetAncfCableUseExactMethod, &PyGeneralContact::SetAncfCableUseExactMethod)\n' 
plr.DefLatexDataAccess('ancfCableUseExactMethod','(default=True) if True, uses exact computation of intersection of 3rd order polynomials and contacting circles ',
                       dataType='bool',
                       )

plr.sPy +=  '        .def_property("ancfCableNumberOfContactSegments", &PyGeneralContact::GetAncfCableNumberOfContactSegments, &PyGeneralContact::SetAncfCableNumberOfContactSegments)\n' 
plr.DefLatexDataAccess('ancfCableNumberOfContactSegments','(default=1) number of segments to be used in case that ancfCableUseExactMethod=False; maximum number of segments=3 ',
                       dataType='int',
                       )

plr.sPy +=  '        .def_property("ancfCableMeasuringSegments", &PyGeneralContact::GetAncfCableMeasuringSegments, &PyGeneralContact::SetAncfCableMeasuringSegments)\n' 
plr.DefLatexDataAccess('ancfCableMeasuringSegments','(default=20) number of segments used to approximate geometry for ANCFCable2D elements for measuring with ShortestDistanceAlongLine; with 20 segments the relative error due to approximation as compared to 10 segments usually stays below 1e-8 ',
                       dataType='int',
                       )


# plr.DefPyFunctionAccess(cClass=classStr, pyName='FinalizeContact', cName='PyFinalizeContact', 
#                                 argList=['mainSystem','searchTreeSize','frictionPairingsInit','searchTreeBoxMin','searchTreeBoxMax'],
#                                 defaultArgs=['','','', '(std::vector<Real>)Vector3D( EXUstd::MAXREAL )','(std::vector<Real>)Vector3D( EXUstd::LOWESTREAL )'],
#                                 description="WILL CHANGE IN FUTURE: Call this function after mbs.Assemble(); precompute some contact arrays (mainSystem needed) and set up necessary parameters for contact: friction, SearchTree, etc.; done after all contacts have been added; function performs checks; empty box will autocompute size!")
plr.DefPyFunctionAccess(cClass=classStr, pyName='SetFrictionPairings', cName='SetFrictionPairings', 
                        argList=['frictionPairings'],
                        example='#set 3 surface friction types, all being 0.1:\\\\gContact.SetFrictionPairings(0.1*np.ones((3,3)));',
                        description="set Coulomb friction coefficients for pairings of materials (e.g., use material 0,1, then the entries (0,1) and (1,0) define the friction coefficients for this pairing); matrix should be symmetric!",
                        argTypes=['ArrayLike'],
                        returnType='None',
                        )

#this could be removed, accessed by variable directly:
plr.DefPyFunctionAccess(cClass=classStr, pyName='SetFrictionProportionalZone', cName='SetFrictionProportionalZone', 
                        argList=['frictionProportionalZone'],
                        description="regularization for friction (m/s); used for all contacts",
                        argTypes=['float'],
                        returnType='None',
                        )
                                   
plr.DefPyFunctionAccess(cClass=classStr, pyName='SetSearchTreeCellSize', cName='SetSearchTreeCellSize', 
                        argList=['numberOfCells'],
                        example='gContact.SetSearchTreeInitSize([10,10,10])',
                        description="set number of cells of search tree (boxed search) in x, y and z direction",
                        argTypes=['[int,int,int]'],
                        returnType='None',
                        )
                                                   
plr.DefPyFunctionAccess(cClass=classStr, pyName='SetSearchTreeBox', cName='SetSearchTreeBox', 
                        argList=['pMin','pMax'],
                        example='gContact.SetSearchTreeBox(pMin=[-1,-1,-1],\\\\ \\TAB pMax=[1,1,1])',
                        description="set geometric dimensions of searchTreeBox (point with minimum coordinates and point with maximum coordinates); if this box becomes smaller than the effective contact objects, contact computations may slow down significantly",
                        argTypes=[vector3D,vector3D],
                        returnType='None',
                       )
                                              
plr.DefPyFunctionAccess(cClass=classStr, pyName='AddSphereWithMarker', cName='AddSphereWithMarker', 
                        argList=['markerIndex','radius','contactStiffness','contactDamping','frictionMaterialIndex'],
                        description="add contact object using a marker (Position or Rigid), radius and contact/friction parameters and return localIndex of the contact item in GeneralContact; frictionMaterialIndex refers to frictionPairings in GeneralContact; contact is possible between spheres (circles in 2D) (if intraSphereContact = True), spheres and triangles and between sphere (=circle) and ANCFCable2D; contactStiffness is computed as serial spring between contacting objects, while damping is computed as a parallel damper",
                        argTypes=['MarkerIndex','float','float','float','int'],
                        returnType='int',
                        )
                                              
plr.DefPyFunctionAccess(cClass=classStr, pyName='AddANCFCable', cName='AddANCFCable', 
                        argList=['objectIndex','halfHeight','contactStiffness','contactDamping','frictionMaterialIndex'],
                        description="add contact object for an ANCF cable element, using the objectIndex of the cable element and the cable's half height as an additional distance to contacting objects (currently not causing additional torque in case of friction), and return localIndex of the contact item in GeneralContact; currently only contact with spheres (circles in 2D) possible; contact computed using exact geometry of elements, finding max 3 intersecting contact regions",
                        argTypes=['ObjectIndex','float','float','float','int'],
                        returnType='int',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='AddTrianglesRigidBodyBased', cName='PyAddTrianglesRigidBodyBased', 
                        argList=['rigidBodyMarkerIndex','contactStiffness','contactDamping','frictionMaterialIndex','pointList','triangleList'],
                        description="add contact object using a rigidBodyMarker (of a body), contact/friction parameters, a list of points (as 3D numpy arrays or lists; coordinates relative to rigidBodyMarker) and a list of triangles (3 indices as numpy array or list) according to a mesh attached to the rigidBodyMarker; returns starting local index of trigsRigidBodyBased at which the triangles are stored; mesh can be produced with GraphicsData2TrigsAndPoints(...); contact is possible between sphere (circle) and Triangle but yet not between triangle and triangle; frictionMaterialIndex refers to frictionPairings in GeneralContact; contactStiffness is computed as serial spring between contacting objects, while damping is computed as a parallel damper (otherwise the smaller damper would always dominate); the triangle normal must point outwards, with the normal of a triangle given with local points (p0,p1,p2) defined as n=(p1-p0) x (p2-p0), see function ComputeTriangleNormal(...)",
                        argTypes=['MarkerIndex','float','float','int','List[[float,float,float]]','List[[int,int,int]]'],
                        returnType='int',
                        )

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#access functions:
plr.DefPyFunctionAccess(cClass=classStr, pyName='GetItemsInBox', cName='PyGetItemsInBox', 
                        argList=['pMin','pMax'],
                        example='gContact.GetItemsInBox(pMin=[0,1,1],\\\\ \\TAB pMax=[2,3,2])',
                        description="Get all items in box defined by minimum coordinates given in pMin and maximum coordinates given by pMax, accepting 3D lists or numpy arrays; in case that no objects are found, False is returned; otherwise, a dictionary is returned, containing numpy arrays with indices of obtained MarkerBasedSpheres, TrigsRigidBodyBased, ANCFCable2D, ...; the indices refer to the local index in GeneralContact which can be evaluated e.g. by GetMarkerBasedSphere(localIndex)",
                        argTypes=[vector3D,vector3D],
                        returnType='Union[dict,bool]',
                        )

#++++++++++++++++++++++++++++++++++++++++++++++++++++

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetSphereMarkerBased', cName='PyGetSphereMarkerBased', 
                        argList=['localIndex','addData'],
                        description="Get dictionary with current position, orientation, velocity, angular velocity as computed in last contact iteration; if addData=True, adds stored data of contact element, such as radius, markerIndex and contact parameters; localIndex is the internal index of contact element, as returned e.g. from GetItemsInBox",
                        argTypes=['int','bool'],
                        defaultArgs=['','False'],
                        returnType='dict',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetSphereMarkerBased', cName='PySetSphereMarkerBased', 
                        argList=['localIndex','contactStiffness','contactDamping','radius','frictionMaterialIndex'],
                        description="Set data of marker based sphere with localIndex (as internally stored) with given arguments; arguments that are < 0 (default) imply that current values are not overwritten",
                        argTypes=['int','float','float','float','int'],
                        defaultArgs=['','-1.','-1.','-1.','-1'],
                        returnType='None',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetTriangleRigidBodyBased', cName='PyGetTriangleRigidBodyBased', 
                        argList=['localIndex'],
                        description="Get dictionary with rigid body index, local position of triangle vertices (nodes) and triangle normal; NOTE: the mesh added to contact is different from this structure, as it contains nodes and connectivity lists; the triangle index corresponds to the order as triangles are added to GeneralContact",
                        argTypes=['int'],
                        defaultArgs=[''],
                        returnType='dict',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetTriangleRigidBodyBased', cName='PySetTriangleRigidBodyBased', 
                        argList=['localIndex','points','contactRigidBodyIndex'],
                        description="Set data of marker based sphere with localIndex (triangle index); points are provided as 3x3 numpy array, with point coordinates in rows; contactRigidBodyIndex<0 indicates no change of the current index (and changing this index should be handled with care)",
                        argTypes=['int',matrix3D,'int'],
                        defaultArgs=['','','-1'],
                        returnType='None',
                        )

#++++++++++++++++++++++++++++++++++++++++++++++++++++

plr.DefPyFunctionAccess(cClass=classStr, pyName='ShortestDistanceAlongLine', cName='PyShortestDistanceAlongLine', 
                        argList=['pStart','direction','minDistance','maxDistance','asDictionary','cylinderRadius','typeIndex'],
                        defaultArgs=['(std::vector<Real>)Vector3D({0,0,0})','(std::vector<Real>)Vector3D({1,0,0})','-1e-7','1e7','False','0','Contact::IndexEndOfEnumList'],
                        description="Find shortest distance to contact objects in GeneralContact along line with pStart (given as 3D list or numpy array) and direction (as 3D list or numpy array with no need to be normalized); the function returns the distance which is >= minDistance and < maxDistance; in case of beam elements, it measures the distance to the beam centerline; the distance is measured from pStart along given direction and can also be negative; if no item is found along line, the maxDistance is returned; if asDictionary=False, the result is a float, while otherwise details are returned as dictionary (including distance, velocityAlongLine (which is the object velocity in given direction and may be different from the time derivative of the distance; works similar to a laser Doppler vibrometer - LDV), itemIndex and itemType in GeneralContact); the cylinderRadius, if not equal to 0, will be used for spheres to find closest sphere along cylinder with given point and direction; the typeIndex can be set to a specific contact type, e.g., which are searched for (otherwise all objects are considered)",
                        argTypes=[vector3D,vector3D,'float','float','bool','float','ContactTypeIndex'],
                        returnType='Union[dict,float]',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='UpdateContacts', cName='PyUpdateContacts', 
                        argList=['mainSystem'],
                        example='gContact.UpdateContacts(mbs)',
                        description="Update contact sets, e.g. if no contact is simulated (isActive=False) but user functions need up-to-date contact states for GetItemsInBox(...) or for GetActiveContacts(...)",
                        argTypes=['MainSystem'],
                        returnType='None',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetActiveContacts', cName='PyGetActiveContacts', 
                        argList=['typeIndex', 'itemIndex'],
                        example='#if explicit solver is used, we first need to update contacts:\\\\gContact.UpdateContacts(mbs)\\\\#obtain active contacts of marker based sphere 42:\\\\gList = gContact.GetActiveContacts(exu.ContactTypeIndex.IndexSpheresMarkerBased, 42)',
                        description="Get list of global item numbers which are in contact with itemIndex of type typeIndex in case that the global itemIndex is smaller than the abs value of the contact pair index; a negative sign indicates that the contacting (spheres) is in Coloumb friction, a positive sign indicates a regularized friction region; in case of itemIndex==-1, it will return the list of numbers of active contacts per item for the contact type; for interpretation of global contact indices, see gContact.GetPythonObject() and documentation; requires either implicit contact computation or UpdateContacts(...) needs to be called prior to this function",
                        argTypes=['ContactTypeIndex','int'],
                        returnType='List[int]',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='GetSystemODE2RhsContactForces', cName='PyGetSystemODE2RhsContactForces', 
                        description="Get numpy array of system vector, containing contribution of contact forces to system ODE2 Rhs vector; contributions to single objects may be extracted by checking the according LTG-array of according objects (such as rigid bodies); the contact forces vector is computed in each contact iteration;",
                        returnType='List[float]',
                        )

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                                              
plr.DefPyFunctionAccess(cClass=classStr, pyName='__repr__', cName='[](const PyGeneralContact &item) {\n            return EXUstd::ToString(item); }', 
                        description="return the string representation of the GeneralContact, containing basic information and statistics",
                        isLambdaFunction = True)


#++++++++++++++++
plr.DefPyFinishClass('GeneralContact')

savedPyi = plr.sPyi+savedPyi
plr.sPyi = ''


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#documentation and pybindings for VisuGeneralContact
classStr = 'VisuGeneralContact'
pyClassStr = 'VisuGeneralContact'
plr.DefPyStartClass(classStr, pyClassStr, 'This structure may contains some visualization parameters in future. '+
                    'Currently, all visualization settings are controlled via SC.visualizationSettings', 
                    subSection=True, labelName='sec:GeneralContact:visualization')

plr.DefLatexStartTable(pyClassStr)

plr.DefPyFunctionAccess(cClass=classStr, pyName='Reset', cName='Reset', 
                        description="reset visualization parameters to default values",
                        returnType='None',
                        )

# plr.sPy +=  '        .def_readwrite("spheresMarkerBasedDraw", &VisuGeneralContact::spheresMarkerBasedDraw, py::return_value_policy::reference)\n' 
# plr.DefLatexDataAccess('spheresMarkerBasedDraw','default = False; if True, markerBased spheres are drawn with given resolution and color ')

# plr.sPy +=  '        .def_readwrite("spheresMarkerBasedResolution", &VisuGeneralContact::spheresMarkerBasedResolution, py::return_value_policy::reference)\n' 
# plr.DefLatexDataAccess('spheresMarkerBasedResolution','default = 4; integer value for number of triangles per circumference of markerBased spheres; higher values leading to smoother spheres but higher graphics costs ')

# plr.sPy +=  '        .def_readwrite("spheresMarkerBasedColor", &VisuGeneralContact::spheresMarkerBasedColor, py::return_value_policy::reference)\n' 
# plr.DefLatexDataAccess('spheresMarkerBasedColor','vector with 4 floats (Float4) for color of markerBased spheres ')

#++++++++++++++++
plr.DefPyFinishClass('GeneralContact')
savedPyi = plr.sPyi+savedPyi
plr.sPyi = ''




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
                    subSection=True, 
                    labelName='sec:MatrixContainer') #section with this label was earlier in theory section

plr.AddDocuCodeBlock(code="""
#Create empty MatrixContainer:
from exudyn import MatrixContainer
mc = MatrixContainer()

#Create MatrixContainer with dense matrix:
#matrix can be a list of lists or a numpy array, e.g.:
matrix = np.eye(6)
mc = MatrixContainer(matrix)

#Set with dense pyArray (a numpy array): 
pyArray = np.array(matrix)
mc.SetWithDenseMatrix(pyArray, useDenseMatrix = True)

#Set empty matrix:
mc.SetWithDenseMatrix([]], bool useDenseMatrix = True)

#Set with list of lists, stored as sparse matrix:
mc.SetWithDenseMatrix([[1,2],[3,4]], bool useDenseMatrix = False)

#Set with sparse CSR matrix:
mc.SetWithSparseMatrixCSR(2,3,[[0,0,13.3],[1,1,4.2],[1,2,42.]], useDenseMatrix=True)

print(mc)
#gives dense matrix:
#[[13.3  0.   0. ]
# [ 0.   4.2 42. ]]
""")

plr.DefLatexStartTable(pyClassStr)

plr.sPy += '        .def(py::init<const py::object&>(), py::arg("matrix"))\n' #constructor with numpy array or list of lists

plr.DefPyFunctionAccess(cClass=classStr, pyName='SetWithDenseMatrix', cName='SetWithDenseMatrix', 
                        argList=['pyArray','useDenseMatrix'],
                        defaultArgs=['','False'],
                        description="set MatrixContainer with dense numpy array of size (n x m); array (=matrix) contains values and matrix size information; if useDenseMatrix=True, matrix will be stored internally as dense matrix, otherwise it will be converted and stored as sparse matrix (which may speed up computations for larger problems)",
                        argTypes=['ArrayLike',''],
                        returnType='None',
                        )
                                              
plr.DefPyFunctionAccess(cClass=classStr, pyName='SetWithSparseMatrixCSR', cName='SetWithSparseMatrixCSR', 
                        argList=['numberOfRowsInit', 'numberOfColumnsInit', 'pyArrayCSR','useDenseMatrix'],
                        defaultArgs=['','','','True'],
                        description="set with sparse CSR matrix format: numpy array 'pyArrayCSR' contains sparse triplet (row, col, value) per row; numberOfRows and numberOfColumns given extra; if useDenseMatrix=True, matrix will be converted and stored internally as dense matrix, otherwise it will be stored as sparse matrix",
                        argTypes=['int','int','ArrayLike',''],
                        returnType='None',
                        )
                                              
plr.DefPyFunctionAccess(cClass=classStr, pyName='GetPythonObject', cName='GetPythonObject', 
                        description="convert MatrixContainer to numpy array (dense) or dictionary (sparse): containing nr. of rows, nr. of columns, numpy matrix with sparse triplets",
                        returnType='Union[dict,ArrayLike]',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='Convert2DenseMatrix', cName='Convert2DenseMatrix', 
                        description="convert MatrixContainer to dense numpy array (SLOW and may fail for too large sparse matrices)",
                        returnType='ArrayLike',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='UseDenseMatrix', cName='UseDenseMatrix', 
                        description="returns True if dense matrix is used, otherwise False",
                        returnType='bool',
                        )

plr.DefPyFunctionAccess(cClass=classStr, pyName='__repr__', cName='[](const PyMatrixContainer &item) {\n            return EXUstd::ToString(item.GetPythonObject()); }', 
                        description="return the string representation of the MatrixContainer",
                        isLambdaFunction = True,
                        )

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
                        description="add single array or list to Vector3DList; array or list must have appropriate dimension!",
                        argTypes=[vector3D],
                        returnType='None',
                        )
                                                                                                            
plr.DefPyFunctionAccess(cClass=classStr, pyName='GetPythonObject', cName='GetPythonObject', 
                       description="convert Vector3DList into (copied) list of numpy arrays",
                       returnType='List['+vector3D+']',
                       )

plr.DefPyFunctionAccess(cClass=classStr, pyName='__len__', 
                       cName='[](const PyVector3DList &item) {\n            return item.NumberOfItems(); }', 
                       description="return length of the Vector3DList, using len(data) where data is the Vector3DList",
                       isLambdaFunction = True,
                       )

plr.DefPyFunctionAccess(cClass=classStr, pyName='__setitem__', 
                       cName='[](PyVector3DList &item, Index index, const py::object& vector) {\n            item.PySetItem(index, vector); }', 
                       description="set list item 'index' with data, write: data[index] = ...",
                       isLambdaFunction = True,
                       )

plr.DefPyFunctionAccess(cClass=classStr, pyName='__getitem__', 
                       cName='[](const PyVector3DList &item, Index index) {\n            return py::array_t<Real>(item[index].NumberOfItems(), item[index].GetDataPointer()); }', 
                       description="get copy of list item with 'index' as vector",
                       isLambdaFunction = True,
                       )
#copy and deepcopy according to Pybind11, see https://pybind11.readthedocs.io/en/latest/advanced/classes.html#pickling-support
# .def("__copy__",  [](const Copyable &self) {
#     return Copyable(self);
# })
# .def("__deepcopy__", [](const Copyable &self, py::dict) {
#     return Copyable(self);
# }, "memo"_a);
plr.DefPyFunctionAccess(cClass=classStr, pyName='__copy__', 
                       cName='[](const PyVector3DList &item) {\n            return PyVector3DList(item); }', 
                       description="copy method to be used for copy.copy(...); in fact does already deep copy",
                       isLambdaFunction = True,
                       )

plr.DefPyFunctionAccess(cClass=classStr, pyName='__deepcopy__', 
                       cName='[](const PyVector3DList &item, py::dict) {\n            return PyVector3DList(item); }, "memo"_a', 
                       description="deepcopy method to be used for copy.copy(...)",
                       isLambdaFunction = True,
                       )

plr.DefPyFunctionAccess(cClass=classStr, pyName='__repr__', 
                       cName='[](const PyVector3DList &item) {\n            return EXUstd::ToString(item.GetPythonObject()); }', 
                       description="return the string representation of the Vector3DList data, e.g.: print(data)",
                       isLambdaFunction = True,
                       )

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
                       description="add single array or list to Vector2DList; array or list must have appropriate dimension!",
                       argTypes=[vector2D],
                       returnType='None',
                       )
                                                                                                    
plr.DefPyFunctionAccess(cClass=classStr, pyName='GetPythonObject', cName='GetPythonObject', 
                       description="convert Vector2DList into (copied) list of numpy arrays",
                       returnType='List['+vector2D+']',
                       )

plr.DefPyFunctionAccess(cClass=classStr, pyName='__len__', 
                       cName='[](const PyVector2DList &item) {\n            return item.NumberOfItems(); }', 
                       description="return length of the Vector2DList, using len(data) where data is the Vector2DList",
                       isLambdaFunction = True,
                       )

plr.DefPyFunctionAccess(cClass=classStr, pyName='__setitem__', 
                       cName='[](PyVector2DList &item, Index index, const py::object& vector) {\n            item.PySetItem(index, vector); }', 
                       description="set list item 'index' with data, write: data[index] = ...",
                       isLambdaFunction = True,
                       )

plr.DefPyFunctionAccess(cClass=classStr, pyName='__getitem__', 
                       cName='[](const PyVector2DList &item, Index index) {\n            return py::array_t<Real>(item[index].NumberOfItems(), item[index].GetDataPointer()); }', 
                       description="get copy of list item with 'index' as vector",
                       isLambdaFunction = True,
                       )

plr.DefPyFunctionAccess(cClass=classStr, pyName='__copy__', 
                       cName='[](const PyVector2DList &item) {\n            return PyVector2DList(item); }', 
                       description="copy method to be used for copy.copy(...); in fact does already deep copy",
                       isLambdaFunction = True,
                       )

plr.DefPyFunctionAccess(cClass=classStr, pyName='__deepcopy__', 
                       cName='[](const PyVector2DList &item, py::dict) {\n            return PyVector2DList(item); }, "memo"_a', 
                       description="deepcopy method to be used for copy.copy(...)",
                       isLambdaFunction = True,
                       )

plr.DefPyFunctionAccess(cClass=classStr, pyName='__repr__', 
                       cName='[](const PyVector2DList &item) {\n            return EXUstd::ToString(item.GetPythonObject()); }', 
                       description="return the string representation of the Vector2DList data, e.g.: print(data)",
                       isLambdaFunction = True,
                       )

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
                       description="add single array or list to Vector6DList; array or list must have appropriate dimension!",
                       argTypes=[vector6D],
                       returnType='None',
                       )
                                                                                                    
plr.DefPyFunctionAccess(cClass=classStr, pyName='GetPythonObject', cName='GetPythonObject', 
                       description="convert Vector6DList into (copied) list of numpy arrays",
                       returnType='List['+vector6D+']',
                       )

plr.DefPyFunctionAccess(cClass=classStr, pyName='__len__', 
                       cName='[](const PyVector6DList &item) {\n            return item.NumberOfItems(); }', 
                       description="return length of the Vector6DList, using len(data) where data is the Vector6DList",
                       isLambdaFunction = True,
                       )

plr.DefPyFunctionAccess(cClass=classStr, pyName='__setitem__', 
                       cName='[](PyVector6DList &item, Index index, const py::object& vector) {\n            item.PySetItem(index, vector); }', 
                       description="set list item 'index' with data, write: data[index] = ...",
                       isLambdaFunction = True,
                       )

plr.DefPyFunctionAccess(cClass=classStr, pyName='__getitem__', 
                       cName='[](const PyVector6DList &item, Index index) {\n            return py::array_t<Real>(item[index].NumberOfItems(), item[index].GetDataPointer()); }', 
                       description="get copy of list item with 'index' as vector",
                       isLambdaFunction = True,
                       )

plr.DefPyFunctionAccess(cClass=classStr, pyName='__copy__', 
                       cName='[](const PyVector6DList &item) {\n            return PyVector6DList(item); }', 
                       description="copy method to be used for copy.copy(...); in fact does already deep copy",
                       isLambdaFunction = True,
                       )

plr.DefPyFunctionAccess(cClass=classStr, pyName='__deepcopy__', 
                       cName='[](const PyVector6DList &item, py::dict) {\n            return PyVector6DList(item); }, "memo"_a', 
                       description="deepcopy method to be used for copy.copy(...)",
                       isLambdaFunction = True,
                       )

plr.DefPyFunctionAccess(cClass=classStr, pyName='__repr__', 
                       cName='[](const PyVector6DList &item) {\n            return EXUstd::ToString(item.GetPythonObject()); }', 
                       description="return the string representation of the Vector6DList data, e.g.: print(data)",
                       isLambdaFunction = True,
                       )

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
                       description="add single 3D array or list of lists to Matrix3DList; array or lists must have appropriate dimension!",
                       argTypes=[matrix3D],
                       returnType='None',
                       )
                                                                                                    
plr.DefPyFunctionAccess(cClass=classStr, pyName='GetPythonObject', cName='GetPythonObject', 
                       description="convert Matrix3DList into (copied) list of 3x3 numpy arrays",
                       returnType='List['+matrix3D+']',
                       )

plr.DefPyFunctionAccess(cClass=classStr, pyName='__len__', 
                       cName='[](const PyMatrix3DList &item) {\n            return item.NumberOfItems(); }', 
                       description="return length of the Matrix3DList, using len(data) where data is the Matrix3DList",
                       isLambdaFunction = True,
                       )

plr.DefPyFunctionAccess(cClass=classStr, pyName='__setitem__', 
                       cName='[](PyMatrix3DList &item, Index index, const py::object& matrix) {\n            item.PySetItem(index, matrix); }', 
                       description="set list item 'index' with matrix, write: data[index] = ...",
                       isLambdaFunction = True,
                       )

plr.DefPyFunctionAccess(cClass=classStr, pyName='__getitem__', 
                       cName='[](const PyMatrix3DList &item, Index index) {\n            return item.PyGetItem(index); }', 
                       description="get copy of list item with 'index' as matrix",
                       isLambdaFunction = True,
                       )

plr.DefPyFunctionAccess(cClass=classStr, pyName='__repr__', 
                       cName='[](const PyMatrix3DList &item) {\n            return EXUstd::ToString(item.GetPythonObject()); }', 
                       description="return the string representation of the Matrix3DList data, e.g.: print(data)",
                       isLambdaFunction = True,
                       )

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
                       description="add single 6D array or list of lists to Matrix6DList; array or lists must have appropriate dimension!",
                       argTypes=[matrix6D],
                       returnType='None',
                       )
                                                                                                    
plr.DefPyFunctionAccess(cClass=classStr, pyName='GetPythonObject', cName='GetPythonObject', 
                       description="convert Matrix6DList into (copied) list of 6x6 numpy arrays",
                       returnType='List['+matrix6D+']',
                       )

plr.DefPyFunctionAccess(cClass=classStr, pyName='__len__', 
                       cName='[](const PyMatrix6DList &item) {\n            return item.NumberOfItems(); }', 
                       description="return length of the Matrix6DList, using len(data) where data is the Matrix6DList",
                       isLambdaFunction = True,
                       )

plr.DefPyFunctionAccess(cClass=classStr, pyName='__setitem__', 
                       cName='[](PyMatrix6DList &item, Index index, const py::object& matrix) {\n            item.PySetItem(index, matrix); }', 
                       description="set list item 'index' with matrix, write: data[index] = ...",
                       isLambdaFunction = True,
                       )

plr.DefPyFunctionAccess(cClass=classStr, pyName='__getitem__', 
                       cName='[](const PyMatrix6DList &item, Index index) {\n            return item.PyGetItem(index); }', 
                       description="get copy of list item with 'index' as matrix",
                       isLambdaFunction = True,
                       )

plr.DefPyFunctionAccess(cClass=classStr, pyName='__repr__', 
                       cName='[](const PyMatrix6DList &item) {\n            return EXUstd::ToString(item.GetPythonObject()); }', 
                       description="return the string representation of the Matrix6DList data, e.g.: print(data)",
                       isLambdaFunction = True,
                       )

#++++++++++++++++
plr.DefPyFinishClass('PyMatrix6DList')

sStubEnums += plr.sPyi #the type definitions are needed earlier and go into enums file

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#now finalize files:
plr.CreateNewRSTfile('TypeDefinitions')
plr.sLatex += sLenum #put latex description of enums after the systemData section
plr.sRST += sRSTenum #put RST description of enums after the systemData section
plr.CreateNewRSTfile('') #this finalizes the list

directoryString = '../Autogenerated/'
pybindFile = directoryString + 'pybind_manual_classes.h'
latexFile = theDocDir+'manual_interfaces.tex'

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
rstDirInt = '../../../docs/RST/cInterface/'
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

    file=io.open(rstDirInt+file+'.rst','w',encoding='utf8')  #clear file by one write access
    file.write(text)
    #file.write(plr.RSTStr())
    file.close()

indexRST += '\n'


file=io.open(rstDirInt+rstIndexFile,'w',encoding='utf8')  #clear file by one write access
file.write(indexRST)
file.close()

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#stub file .pyi (will be merged with general file)
file=io.open('generated/stubAutoBindings.pyi','w',encoding='utf8')  #clear file by one write access
file.write(savedPyi)
#file.write(plr.sPyi)
file.close()

#stub file symbolic.pyi (will be merged with general file)
file=io.open('generated/stubSymbolic.pyi','w',encoding='utf8')  #clear file by one write access
file.write(plrsym.sPyi)
#file.write(plr.sPyi)
file.close()

file=io.open('generated/stubEnums.pyi','w',encoding='utf8')  #clear file by one write access
file.write(sStubEnums)
file.close()

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#write function, class, ... names for conf.py
exuDir = '../../../docs/RST/'

sConfHelper = ''
sConfHelper += '#this is a helper file to define additional keywords for examples\n'
sConfHelper += '#Created: 2023-03-17, Johannes Gerstmayr\n\n'

#some manual entries ...
localListClassNames.append('SimulationSettings')
localListFunctionNames.append('visualizationSettings') #in fact, it is a member
localListFunctionNames.append('systemData') #in fact, it is a member
localListFunctionNames.append('systemIsConsistent') #in fact, it is a member
localListFunctionNames.append('interactiveMode') #in fact, it is a member

#list of classes and enum classes:
sConfHelper += 'listClassNames=['
for s in localListClassNames:
    sConfHelper += "'" + s + "'" + ', '
for s in localListEnumNames:
    sConfHelper += "'" + s + "'" + ', '
sConfHelper += ']\n\n'

sConfHelper += 'listFunctionNames=['
for s in localListFunctionNames:
    sConfHelper += "'" + s + "'" + ', '
sConfHelper += ']\n\n'


with open(exuDir+'confHelper.py', 'w') as f:
    f.write(sConfHelper)



