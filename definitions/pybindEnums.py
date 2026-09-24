#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN definition file
#
# Details:  the enumeration types of the exudyn module (OutputVariableType, ConfigurationType, ...);
#           their stubs go into stubEnums.pyi, their documentation after the data structures.
#           The calls are recorded by PybindInterface (pybindTypes.py) and replayed by
#           tools/generators/pybindEmitter.py into pybind_manual_classes.h, the stub fragments and
#           the Python-C++ interface documentation.
#
# Author:   Johannes Gerstmayr
# Date:     2018-05-18 (created in autoGeneratePyBindings.py), 2026-09-14 (moved to definitions/)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

from pybindTypes import *

pb = PybindInterface()
from outputVariableTypes import outputVariableTypes
from enumTypes import enumTypes

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
#synchronized by hand
for outputVariable in outputVariableTypes:
    pb.AddEnumValue(pyClass, outputVariable.name, outputVariable.description)

pb.CppCode('		'+enumExportValues+';\n\n')
pb.DefLatexFinishTable()

#the other enums: their values, the C++ enums and string functions all come from
#definitions/enumTypes.py
for enum in enumTypes:
    if enum.description is None:
        continue #C++ only
    cClass = enum.Namespace() if enum.Namespace() != '' else enum.pythonName
    if enum.Namespace() != '':
        pb.DefStartEnumClass(className=enum.pythonName, description=enum.description,
                             subSection=True, labelName='sec:'+enum.pythonName, cClass=enum.cppName)
    else:
        pb.DefStartEnumClass(className=enum.pythonName, description=enum.description,
                             subSection=True, labelName='sec:'+enum.pythonName)
    pb.DefLatexStartTable(enum.pythonName)
    for value in enum.values:
        if value.python:
            pb.AddEnumValue(cClass, value.name, value.description)
    pb.CppCode('		'+enumExportValues+';\n\n')
    pb.DefLatexFinishTable()

