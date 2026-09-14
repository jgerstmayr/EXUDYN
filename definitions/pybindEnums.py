#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN definition file
#
# Details:  the enumeration types of the exudyn module (OutputVariableType, ConfigurationType, ...);
#           their stubs go into stubEnums.pyi, their documentation after the data structures.
#           The calls are recorded by PybindInterface (pybindTypes.py) and replayed by
#           tools/generators/pybindEmitter.py into pybind_manual_classes.h, the stub fragments and
#           the Python-C++ interface documentation (revision plan step 33, part 2d).
#
# Author:   Johannes Gerstmayr
# Date:     2018-05-18 (created in autoGeneratePyBindings.py), 2026-09-14 (moved to definitions/)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

from pybindTypes import *

pb = PybindInterface()
from outputVariableTypes import outputVariableTypes

pb.AddDocu(text='This section defines a couple of structures (C++: enum aka enumeration type), which are used to select, e.g., a configuration type or a variable type. In the background, these types are integer numbers, but for safety, the types should be used as type variables. See this examples:\n\n', 
            section='Type definitions', sectionLevel=1,sectionLabel='sec:cinterface:typedef')

#sLenum = '\section{}\n \n\n'
pb.AddDocuCodeBlock("""
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

pb.CppCode('\n//        pybinding to enum classes:\n')


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++
pyClass = 'OutputVariableType'

descriptionStr = 'The enumeration type  ' + pyClass + ' is used for selecting output values, e.g., for GetObjectOutput(...) or for selecting variables for contour plot.\n\n'
descriptionStr += 'Available output variables and the interpreation of the output variable can be found at the object definitions. \n The OutputVariableType does not provide information about the size of the output variable, which can be either scalar or a list (vector). For vector output quantities, the contour plot option offers an additional parameter for selection of the component of the OutputVariableType. The components are usually out of \\{0,1,2\\}, representing \\{x,y,z\\} components (e.g., of displacements, velocities, ...), or \\{0,1,2,3,4,5\\} representing \\{xx,yy,zz,yz,xz,xy\\} components (e.g., of strain or stress). In order to compute a norm, chose component=-1, which will result in the quadratic norm for other vectors and to a norm specified for stresses (if no norm is defined for an outputVariable, it does not compute anything)\n'

pb.DefStartEnumClass(className = pyClass, 
                        description=descriptionStr, 
                        subSection=True, labelName='sec:'+pyClass)
pb.DefLatexStartTable(pyClass)

#the enum, its bit positions, the two C++ helper functions and this Python/documentation
#table all come from definitions/outputVariableTypes.py - nothing is left to keep
#synchronized by hand (revision plan step 31d)
for outputVariable in outputVariableTypes:
    pb.AddEnumValue(pyClass, outputVariable.name, outputVariable.description)

pb.CppCode('		'+enumExportValues+';\n\n')
pb.DefLatexFinishTable()

#+++++++++++++++++++++++++++++++++++++++++++++++++++
pyClass = 'ConfigurationType'

descriptionStr = 'The enumeration type  ' + pyClass + ' is used for selecting a configuration for reading or writing information to the module. Specifically, the ConfigurationType.Current configuration is usually used at the end of a solution process, to obtain result values, or the ConfigurationType.Initial is used to set initial values for a solution process.\n\n'

pb.DefStartEnumClass(className = pyClass, 
                            description=descriptionStr, 
                            subSection=True, labelName='sec:'+pyClass)
pb.DefLatexStartTable(pyClass)
#keep this list synchronized with the accoring enum structure in C++!!!
pb.AddEnumValue(pyClass, '_None', 'no configuration; usually not valid, but may be used, e.g., if no configurationType is required')
pb.AddEnumValue(pyClass, 'Initial', 'initial configuration prior to static or dynamic solver; is computed during mbs.Assemble() or AssembleInitializeSystemCoordinates()')
pb.AddEnumValue(pyClass, 'Current', 'current configuration during and at the end of the computation of a step (static or dynamic)')
pb.AddEnumValue(pyClass, 'Reference', 'configuration used to define deformable bodies (reference configuration for finite elements) or joints (configuration for which some joints are defined)')
pb.AddEnumValue(pyClass, 'StartOfStep', 'during computation, this refers to the solution at the start of the step = end of last step, to which the solver falls back if convergence fails')
pb.AddEnumValue(pyClass, 'Visualization', 'this is a state completely de-coupled from computation, used for visualization')
pb.AddEnumValue(pyClass, 'EndOfEnumList', 'this marks the end of the list, usually not important to the user')

pb.CppCode('		'+enumExportValues+';\n\n')
pb.DefLatexFinishTable()

#+++++++++++++++++++++++++++++++++++++++++++++++++++
pyClass = 'ItemType'

descriptionStr = 'The enumeration type  ' + pyClass + ' is used for defining types of indices, e.g., in render window and will be also used in item dictionaries in future.\n\n'

pb.DefStartEnumClass(className = pyClass, 
                            description=descriptionStr, 
                            subSection=True, labelName='sec:'+pyClass)
pb.DefLatexStartTable(pyClass)
#keep this list synchronized with the accoring enum structure in C++!!!
pb.AddEnumValue(pyClass, '_None', 'item has no type')
pb.AddEnumValue(pyClass, 'Node', 'item or index is of type Node')
pb.AddEnumValue(pyClass, 'Object', 'item or index is of type Object')
pb.AddEnumValue(pyClass, 'Marker', 'item or index is of type Marker')
pb.AddEnumValue(pyClass, 'Load', 'item or index is of type Load')
pb.AddEnumValue(pyClass, 'Sensor', 'item or index is of type Sensor')

pb.CppCode('		'+enumExportValues+';\n\n')
pb.DefLatexFinishTable()

#+++++++++++++++++++++++++++++++++++++++++++++++++++
pyClass = 'NodeType'
cClass = 'Node'

descriptionStr = 'The enumeration type  ' + pyClass + ' is used for defining node types for 3D rigid bodies.\n\n'

pb.DefStartEnumClass(className = pyClass, 
                            description=descriptionStr, 
                            subSection=True, labelName='sec:'+pyClass, cClass=cClass + '::Type')
pb.DefLatexStartTable(pyClass)
#keep this list synchronized with the accoring enum structure in C++!!!
pb.AddEnumValue(cClass, '_None', 'node has no type')
pb.AddEnumValue(cClass, 'Ground', 'ground node')
pb.AddEnumValue(cClass, 'Position2D', '2D position node ')
pb.AddEnumValue(cClass, 'Orientation2D', 'node with 2D rotation')
pb.AddEnumValue(cClass, 'Point2DSlope1', '2D node with 1 slope vector')
pb.AddEnumValue(cClass, 'Position', '3D position node')
pb.AddEnumValue(cClass, 'Orientation', '3D orientation node')
pb.AddEnumValue(cClass, 'RigidBody', 'node that can be used for rigid bodies')
pb.AddEnumValue(cClass, 'RotationEulerParameters', 'node with 3D orientations that are modelled with Euler parameters (unit quaternions)')
pb.AddEnumValue(cClass, 'RotationRxyz', 'node with 3D orientations that are modelled with Tait-Bryan angles')
pb.AddEnumValue(cClass, 'RotationRotationVector', 'node with 3D orientations that are modelled with the rotation vector')
pb.AddEnumValue(cClass, 'LieGroupWithDirectUpdate', 'node to be solved with Lie group methods, without data coordinates')
#pb.AddEnumValue(cClass, 'LieGroupWithDataCoordinates', 'node to be solved with Lie group methods, having data coordinates')
pb.AddEnumValue(cClass, 'GenericODE2', 'node with general ODE2 variables')
pb.AddEnumValue(cClass, 'GenericODE1', 'node with general ODE1 variables')
pb.AddEnumValue(cClass, 'GenericAE', 'node with general algebraic variables')
pb.AddEnumValue(cClass, 'GenericData', 'node with general data variables')
pb.AddEnumValue(cClass, 'PointSlope1', 'node with 1 slope vector')
pb.AddEnumValue(cClass, 'PointSlope12', 'node with 2 slope vectors in x and y direction')
pb.AddEnumValue(cClass, 'PointSlope23', 'node with 2 slope vectors in y and z direction')


pb.CppCode('		'+enumExportValues+';\n\n')
pb.DefLatexFinishTable()

#+++++++++++++++++++++++++++++++++++++++++++++++++++
pyClass = 'JointType'
cClass = 'Joint'

descriptionStr = 'The enumeration type  ' + pyClass + ' is used for defining joint types, used in KinematicTree.\n\n'

pb.DefStartEnumClass(className = pyClass, 
                      description=descriptionStr, 
                      subSection=True, labelName='sec:'+pyClass, cClass=cClass + '::Type')
pb.DefLatexStartTable(pyClass)
#keep this list synchronized with the accoring enum structure in C++!!!
pb.AddEnumValue(cClass, '_None', 'node has no type')

pb.AddEnumValue(cClass, 'RevoluteX', 'revolute joint type with rotation around local X axis')
pb.AddEnumValue(cClass, 'RevoluteY', 'revolute joint type with rotation around local Y axis')
pb.AddEnumValue(cClass, 'RevoluteZ', 'revolute joint type with rotation around local Z axis')
pb.AddEnumValue(cClass, 'PrismaticX', 'prismatic joint type with translation along local X axis')
pb.AddEnumValue(cClass, 'PrismaticY', 'prismatic joint type with translation along local Y axis')
pb.AddEnumValue(cClass, 'PrismaticZ', 'prismatic joint type with translation along local Z axis')

pb.CppCode('		'+enumExportValues+';\n\n')
pb.DefLatexFinishTable()

#+++++++++++++++++++++++++++++++++++++++++++++++++++
pyClass = 'DynamicSolverType'

descriptionStr = 'The enumeration type  ' + pyClass + ' is used for selecting dynamic solvers for simulation.\n\n'

pb.DefStartEnumClass(className = pyClass, 
                            description=descriptionStr, 
                            subSection=True, labelName='sec:'+pyClass)
pb.DefLatexStartTable(pyClass)
#keep this list synchronized with the accoring enum structure in C++!!!
pb.AddEnumValue(pyClass, 'GeneralizedAlpha', 'an implicit solver for index 3 problems; intended to be used for solving directly the index 3 constraints using the spectralRadius sufficiently small (usually 0.5 .. 1)')
pb.AddEnumValue(pyClass, 'TrapezoidalIndex2', 'an implicit solver for index 3 problems with index2 reduction; uses generalized alpha solver with settings for Newmark with index2 reduction')
pb.AddEnumValue(pyClass, 'ExplicitEuler',    'an explicit 1st order solver (generally not compatible with constraints)')
pb.AddEnumValue(pyClass, 'ExplicitMidpoint', 'an explicit 2nd order solver (generally not compatible with constraints)')
pb.AddEnumValue(pyClass, 'RK33',     'an explicit 3 stage 3rd order Runge-Kutta method, aka "Heun third order"; (generally not compatible with constraints)')
pb.AddEnumValue(pyClass, 'RK44',     'an explicit 4 stage 4th order Runge-Kutta method, aka "classical Runge Kutta" (generally not compatible with constraints), compatible with Lie group integration and elimination of CoordinateConstraints')
pb.AddEnumValue(pyClass, 'RK67',     "an explicit 7 stage 6th order Runge-Kutta method, see 'On Runge-Kutta Processes of High Order', J. C. Butcher, J. Austr Math Soc 4, (1964); can be used for very accurate (reference) solutions, but without step size control!")
pb.AddEnumValue(pyClass, 'ODE23',    'an explicit Runge Kutta method with automatic step size selection with 3rd order of accuracy and 2nd order error estimation, see Bogacki and Shampine, 1989; also known as ODE23 in MATLAB')
pb.AddEnumValue(pyClass, 'DOPRI5',   "an explicit Runge Kutta method with automatic step size selection with 5th order of accuracy and 4th order error estimation, see  Dormand and Prince, 'A Family of Embedded Runge-Kutta Formulae.', J. Comp. Appl. Math. 6, 1980")
pb.AddEnumValue(pyClass, 'DVERK6', '[NOT IMPLEMENTED YET] an explicit Runge Kutta solver of 6th order with 5th order error estimation; includes adaptive step selection')
pb.AddEnumValue(pyClass, 'VelocityVerlet', "[TEST phase] a special explicit time integration scheme, the 'velocity Verlet' method (similar to leap frog method), with second order accuracy for conservative second order differential equations, often used for particle dynamics and contact; implementation uses Explicit Euler for ODE1 equations")

pb.CppCode('		'+enumExportValues+';\n\n')
pb.DefLatexFinishTable()

#+++++++++++++++++++++++++++++++++++++++++++++++++++
pyClass = 'CrossSectionType'

descriptionStr = 'The enumeration type  ' + pyClass + ' is used for defining beam cross section types.\n\n'

pb.DefStartEnumClass(className = pyClass, 
                        description=descriptionStr, 
                        subSection=True, labelName='sec:'+pyClass)
pb.DefLatexStartTable(pyClass)
#keep this list synchronized with the accoring enum structure in C++!!!
pb.AddEnumValue(pyClass, 'Polygon', 'cross section profile defined by polygon')
pb.AddEnumValue(pyClass, 'Circular', 'cross section is circle or elliptic')

pb.CppCode('		'+enumExportValues+';\n\n')
pb.DefLatexFinishTable()

#+++++++++++++++++++++++++++++++++++++++++++++++++++
pyClass = 'KeyCode'

descriptionStr = 'The enumeration type  ' + pyClass + ' is used for special key codes in keyPressUserFunction.\n\n'

pb.DefStartEnumClass(className = pyClass, 
                            description=descriptionStr, 
                            subSection=True, labelName='sec:'+pyClass)
pb.DefLatexStartTable(pyClass)
#keep this list synchronized with the accoring enum structure in C++!!!
pb.AddEnumValue(pyClass, 'SPACE', 'space key')
pb.AddEnumValue(pyClass, 'ENTER', 'enter (return) key')
pb.AddEnumValue(pyClass, 'TAB',   '')
pb.AddEnumValue(pyClass, 'BACKSPACE', '')
pb.AddEnumValue(pyClass, 'RIGHT', 'cursor right')
pb.AddEnumValue(pyClass, 'LEFT', 'cursor left')
pb.AddEnumValue(pyClass, 'DOWN', 'cursor down')
pb.AddEnumValue(pyClass, 'UP', 'cursor up')
pb.AddEnumValue(pyClass, 'F1', 'function key F1')
pb.AddEnumValue(pyClass, 'F2', 'function key F2')
pb.AddEnumValue(pyClass, 'F3', 'function key F3')
pb.AddEnumValue(pyClass, 'F4', 'function key F4')
pb.AddEnumValue(pyClass, 'F5', 'function key F5')
pb.AddEnumValue(pyClass, 'F6', 'function key F6')
pb.AddEnumValue(pyClass, 'F7', 'function key F7')
pb.AddEnumValue(pyClass, 'F8', 'function key F8')
pb.AddEnumValue(pyClass, 'F9', 'function key F9')
pb.AddEnumValue(pyClass, 'F10', 'function key F10')

pb.CppCode('		'+enumExportValues+';\n\n')
pb.DefLatexFinishTable()

#+++++++++++++++++++++++++++++++++++++++++++++++++++
pyClass = 'LinearSolverType'


descriptionStr = 'The enumeration type  ' + pyClass + ' is used for selecting linear solver types, which are dense or sparse solvers.\n\n'

pb.DefStartEnumClass(className = pyClass, 
                            description=descriptionStr, 
                            subSection=True, labelName='sec:'+pyClass)
pb.DefLatexStartTable(pyClass)
#keep this list synchronized with the accoring enum structure in C++!!!
pb.AddEnumValue(pyClass, '_None', 'no value; used, e.g., if no solver is selected')
pb.AddEnumValue(pyClass, 'EXUdense', 'use dense matrices and according solvers for densly populated matrices (usually the CPU time grows cubically with the number of unknowns)')
pb.AddEnumValue(pyClass, 'EigenSparse', 'use sparse matrices and according solvers; additional overhead for very small multibody systems; specifically, memory allocation is performed during a factorization process')
pb.AddEnumValue(pyClass, 'EigenSparseSymmetric', 'use sparse matrices and according solvers; NOTE: this is the symmetric mode, which assumes symmetric system matrices; this is EXPERIMENTAL and should only be used of user knows that the system matrices are (nearly) symmetric; does not work with scaled GeneralizedAlpha matrices; does not work with constraints, as it must be symmetric positive definite')
pb.AddEnumValue(pyClass, 'EigenDense', "use Eigen's LU factorization with partial pivoting (faster than EXUdense) or full pivot (if linearSolverSettings.ignoreSingularJacobian=True; is much slower, but can resolve overdetermined and underdetermined problems!)")

pb.CppCode('		'+enumExportValues+';\n\n')
pb.DefLatexFinishTable()


#+++++++++++++++++++++++++++++++++++++++++++++++++++
pyClass = 'ContactTypeIndex'
cClass = 'Contact'

descriptionStr = 'The enumeration type  ' + pyClass + ' is used in GeneralContact to select specific contact items, such as spheres, ANCFCable or triangle items.\n\n'

pb.DefStartEnumClass(className = pyClass, 
                            description=descriptionStr, 
                            subSection=True, labelName='sec:'+pyClass, cClass=cClass + '::TypeIndex')
pb.DefLatexStartTable(pyClass)
#keep this list synchronized with the accoring enum structure in C++!!!
pb.AddEnumValue(cClass, 'IndexSpheresMarkerBased', 'spheres attached to markers')
pb.AddEnumValue(cClass, 'IndexANCFCable2D', 'ANCFCable2D contact items')
pb.AddEnumValue(cClass, 'IndexTrigsRigidBodyBased', 'triangles attached to rigid body (or rigid body marker)')
pb.AddEnumValue(cClass, 'IndexEndOfEnumList', 'signals end of list')

pb.CppCode('		'+enumExportValues+';\n\n')
pb.DefLatexFinishTable()
