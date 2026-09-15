#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  Emits python/exudyn/itemInterface.py - the Python classes (ObjectMassPoint,
#           VObjectMassPoint, ...) that users instantiate to build item dictionaries - directly
#           from definitions/. The first emitter split out of pythonAutoGenerateObjects.py
#           (revision plan step 33, part 2b); the code was moved from there, so the output is
#           byte-identical to what the monolith wrote.
#
# Usage:    python tools/generators/itemInterfaceEmitter.py [--output FILE]
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-14 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import argparse
import io
import os
import sys

toolsDirectory = os.path.dirname(os.path.abspath(__file__))
if toolsDirectory not in sys.path:
    sys.path.insert(0, toolsDirectory)

import itemModel as im                                                              # noqa: E402
import typeModel as tm                                                              # noqa: E402
import publicApi                                                                     # noqa: E402
from itemModel import (pyFunctionTypeConversion, IsAVector,                         # noqa: E402
                       IsASimpleMatrix, IsAArrayIndex, IsTypeWithRangeCheck, ExtractLatexSymbol,
                       possibleTypes)
from autoGenerateHelper import (DefaultValue2Python, GetTypesStringLatex,           # noqa: E402
                                SplitSummaryDescription, GoogleDocstringRenderer,
                                CleanStringForPyiDescription)

ADD_DOCSTRINGS = True
space4 = '    '


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def CreateStringSymbolicUserFunctionArgs(pySymbolicUserFunction):
    """dictionary converting item-userfunction strings into the user function argument list"""
    userFunctionArgsDict = {}

    for item in pySymbolicUserFunction:
        if len(item) == 0: continue

        itemType = item['itemType'] #ConnectorSpringDamper
        classType = item['classType']   #Object, Node
        userFunctionName = item['userFunctionName']
        pyUserFunctionType = item['pyUserFunctionType']

        #create string for function named args
        fcnArgs = pyFunctionTypeConversion[pyUserFunctionType].split('(')[1].split(')')[0].split(',')
        fcnType = pyFunctionTypeConversion[pyUserFunctionType].split('(')[0].split('<')[1].strip()
        fcnArgsList = ['mbs']
        fcnTypesList = ['MainSystem']

        cnt = 0
        for arg in fcnArgs[1:]: #omit MainSystem
            fcnArgsList += ['arg'+str(cnt)]
            fcnTypesList += [arg.strip()]
            cnt+=1

        userFunctionArgsDict[classType+itemType+','+userFunctionName] = [fcnTypesList,fcnArgsList,[fcnType]]

    return userFunctionArgsDict


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def ItemDocstrings(definition):
    """the Google-style docstring data of the item class and of its visualization class"""
    className = definition['className']
    classType = definition.get('classType', '')
    (pyiSummary, pyiDescription) = SplitSummaryDescription(CleanStringForPyiDescription(
        im.ClassDescription(definition)))
    dataDocstring = {'kind': 'classFunction', 'notes': [], 'inputs': []}
    dataDocstring['summary'] = pyiSummary
    dataDocstring['description'] = pyiDescription
    dataDocstringV = {'kind': 'classFunction',
                      'summary': 'Visualization data for ' + className,
                      'inputs': []}

    requestedMarkerString = ''
    itemTypeString = ''
    requestedNodeString = ''
    for member in definition['members']:
        if im.IsInterfaceParameter(member):
            [parameterDescription, latexSymbol] = ExtractLatexSymbol(im.Description(member))
            thisDataDocString = dataDocstringV if 'V' in im.Destination(member) else dataDocstring
            thisDataDocString['inputs'].append({'name': member['pythonName'],
                                                'description': CleanStringForPyiDescription(parameterDescription),
                                                'type_hint': tm.Render(im.TypeName(member), 'pyTyping', 'items')
                                                })
        elif member['pythonName'] == 'GetRequestedMarkerType':
            requestedMarkerString = GetTypesStringLatex(im.DefaultValueString(member), 'Marker',
                                                        possibleTypes['Marker'], ' +')
        elif member['pythonName'] == 'GetRequestedNodeType':
            requestedNodeString = GetTypesStringLatex(im.DefaultValueString(member), 'Node',
                                                      possibleTypes['Node'], ' +')
        elif member['pythonName'] == 'GetType':
            searchType = classType
            if classType == 'Object': searchType += 'Type'
            itemTypeString = GetTypesStringLatex(im.DefaultValueString(member), searchType,
                                                 possibleTypes[classType])

    pythonShortName = definition.get('pythonShortName', '') or ''
    if len(requestedMarkerString) + len(itemTypeString) + len(pythonShortName) != 0:
        if len(itemTypeString) != 0:
            dataDocstring['notes'].append(classType + ' has/provides the following types: '
                                          + CleanStringForPyiDescription(itemTypeString))
        if len(requestedMarkerString) != 0:
            dataDocstring['notes'].append('Requested Marker type: '
                                          + CleanStringForPyiDescription(requestedMarkerString))
        if len(requestedNodeString) != 0 and requestedNodeString.find('_None') == -1:
            dataDocstring['notes'].append('Requested Node type: '
                                          + CleanStringForPyiDescription(requestedNodeString))

    return dataDocstring, dataDocstringV


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def ItemClasses(definition):
    """the V<Item> and <Item> classes of one item, '' if it has no Python interface"""
    if not im.HasPythonInterface(definition):
        return ''

    className = definition['className']
    classTypeStr = definition.get('classType', '')
    sTypeName = className.replace(classTypeStr, '')
    dataDocstring, dataDocstringV = ItemDocstrings(definition)

    sPythonClass = '' #the python interface class definition
    sPythonClassInit = '' #the init function body
    sPythonIter = ''  #the iterator member function

    vPythonClass = '' #the python visualization interface class definition
    vPythonClassInit = '' #the init function body
    vPythonIter = ''  #the iterator member function
    sIndent = space4 #4 spaces indentation for python

    renderer = GoogleDocstringRenderer()

    sPythonClass += 'class ' + className + ':\n'
    if ADD_DOCSTRINGS: sPythonClass += renderer.render(dataDocstring, indent=sIndent)+'\n'
    sPythonClass += sIndent+'def __init__(self'
    sPythonIter += sIndent+sIndent+'yield ' + "'" + classTypeStr[0].lower() + classTypeStr[1:] + 'Type' + "'" + ', ' + "'"+sTypeName+"'" + '\n'

    vPythonClass += 'class V' + className + ':\n'
    if ADD_DOCSTRINGS: vPythonClass += renderer.render(dataDocstringV, indent=sIndent)+'\n'
    vPythonClass += sIndent+'def __init__(self'
    vDefaultDict = '{'
    vDefaultDictEmpty = True

    for member in definition['members']:
        if im.IsInterfaceParameter(member) and not im.IsReadOnly(member):
            typeName = im.TypeName(member)
            pythonName = member['pythonName']
            sString = ''
            if (typeName == 'String'):
                sString="'"

            defaultValueStr = sString+DefaultValue2Python(im.DefaultValueString(member))+sString

            #special treatment of BodyGraphicsData
            if typeName == 'BodyGraphicsData' or typeName == 'BodyGraphicsDataList':
                defaultValueStr = '[]'

            #write item interface class initialization, constructor and iterator doc:
            tempVPythonDict = "'" + pythonName + "': "
            tempPythonClass = ', ' + pythonName
            if len(defaultValueStr) != 0:
                tempPythonClass += ' = ' + defaultValueStr
                tempVPythonDict += defaultValueStr
            else:
                tempVPythonDict += "None"

            #range checks are done in C++ on every write path (revision plan step 34c4 b)
            parameterWithCheck = pythonName
            if (IsAVector(typeName)
                or IsASimpleMatrix(typeName)
                ):
                if typeName == 'NumpyVector':
                    parameterWithCheck = 'CheckForValidNumpyArray('+parameterWithCheck+')'
                elif typeName == 'NumpyMatrix':
                    parameterWithCheck = 'CheckForValidNumpyArray('+parameterWithCheck+')'
                else:
                    parameterWithCheck = 'np.array('+parameterWithCheck+')'
            elif (IsAArrayIndex(typeName)
                  or typeName == 'BodyGraphicsData' #in this case, flat copy is ok
                  or typeName == 'BodyGraphicsDataList' #in this case, flat copy is ok
                  or typeName == 'JointTypeList'
                  or typeName == 'ArrayIndex'
                  ):
                parameterWithCheck = 'copy.copy('+parameterWithCheck+')' #flat copy is sufficient
            elif defaultValueStr.strip().startswith('['):
                print('WARNING: unresolved default [...] with', pythonName)
            elif defaultValueStr.strip().startswith('{'): #for now, only visualization
                print('WARNING: unresolved default {...} with', pythonName)

            tempPythonClassInit = sIndent+sIndent+'self.' + pythonName + ' = ' + parameterWithCheck + '\n'
            tempPythonIter = sIndent+sIndent+'yield ' + "'" + pythonName + "'" + ', self.' + pythonName + '\n'

            if 'V' in im.Destination(member): #visualization
                vPythonClass += tempPythonClass
                vPythonClassInit += tempPythonClassInit
                vPythonIter += tempPythonIter
                if not(vDefaultDictEmpty): #if already second dict entry added, also add a comma separator
                    vDefaultDict += ", "

                vDefaultDict += tempVPythonDict
                vDefaultDictEmpty = False
                sPythonIter += sIndent+sIndent+'yield ' + "'V" + pythonName + "'" + ', dict(self.visualization)["' + pythonName + '"]\n'
            else: #rest: computational or main
                sPythonClass += tempPythonClass
                sPythonClassInit += tempPythonClassInit
                sPythonIter += tempPythonIter

    vDefaultDict += '}'
    sPythonClass += ', visualization = ' + vDefaultDict + '):\n' #add visualization structure (must always be there...)
    sPythonClass += sPythonClassInit + sIndent+sIndent+'self.visualization = CopyDictLevel1(visualization)\n\n'
    sPythonClass += sIndent+'def __iter__(self):\n'
    sPythonClass += sPythonIter + '\n'
    sPythonClass += sIndent+'def __repr__(self):\n'
    sPythonClass += sIndent+space4+'return str(dict(self))\n'
    sPythonClass += '\n' #one empty line at end of class

    vPythonClass += '):\n'
    vPythonClass += vPythonClassInit + '\n'
    vPythonClass += sIndent+'def __iter__(self):\n'
    vPythonClass += vPythonIter + '\n'
    vPythonClass += sIndent+'def __repr__(self):\n'
    vPythonClass += sIndent+space4+'return str(dict(self))\n'
    vPythonClass += '\n' #one empty line at end of class
    sPythonClass = vPythonClass + sPythonClass #visualization class must be first, otherwise the main class cannot be initialized
    pythonShortName = definition.get('pythonShortName', '') or ''
    if len(pythonShortName):
        sPythonClass += '#add typedef for short usage:\n'
        sPythonClass += pythonShortName + ' = ' + className + '\n'
        sPythonClass += 'V' + pythonShortName + ' = V' + className + '\n\n'

    return sPythonClass


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def EmitItemInterface(definitions):
    """the complete text of itemInterface.py"""
    symbolicUserFunctionSet = list(im.mainSystemUserFunctions)
    for definition in definitions:
        symbolicUserFunctionSet += im.SymbolicUserFunctions(definition)
    userFunctionArgsDict = CreateStringSymbolicUserFunctionArgs(symbolicUserFunctionSet)

    s = fileHeader
    s += '\nuserFunctionArgsDict = ' + str(userFunctionArgsDict).replace(']],',']],\n       ') + '\n\n\n'

    for classType in im.itemTypeOrder:
        s += '#+++++++++++++++++++++++++++++++\n#' + classType.upper() + '\n'
        for definition in definitions:
            if definition.get('classType', '') == classType:
                s += ItemClasses(definition)

    #__all__ after the imports, from the same rule tools/checkAll.py checks (step 107c)
    marker = '\n\n#helper function for level-1 copy of dicts'
    s = s.replace(marker, '\n\n' + publicApi.AllText(publicApi.PublicNames(s)) + marker, 1)
    return s


fileHeader = '''#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is the Exudyn item interface
# 
# Details:  automatically generated file for conversion of item (node, object, marker, ...) data to dictionaries
# 
# Author:   Johannes Gerstmayr
# Date:     2019-07-01 (first created)
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn #for exudyn.InvalidIndex() and other exudyn native structures needed in RigidBodySpringDamper
import numpy as np
import copy \n

#helper function for level-1 copy of dicts (for visualization default args!)
#visualization dictionaries (which may be huge, are only flat copied, which is sufficient)
def CopyDictLevel1(originalDict):
    if isinstance(originalDict,dict): #copy only required if default dict is used
        copyDict = {}
        for key, value in originalDict.items():
            copyDict[key] = copy.copy(value)
        return copyDict
    else:
        return originalDict #fast track for everything else

#helper function diagonal matrices, not needing numpy
def IIDiagMatrix(rowsColumns, value):
    m = []
    for i in range(rowsColumns):
        m += [rowsColumns*[0]]
        m[i][i] = value
    return m\n
    
#helper function to check valid range
def CheckForValidUInt(value, parameterName, objectName):
    if value < 0:
        raise ValueError("Error in "+objectName+": (int) parameter "+parameterName + " may not be negative, but received "+str(value))
        return 0
    return value

#helper function to check valid range
def CheckForValidPInt(value, parameterName, objectName):
    if value <= 0:
        raise ValueError("Error in "+objectName+": (int) parameter "+parameterName + " must be positive (> 0), but received "+str(value))
        return 1 #this position is usually not reached
    return value
    
#helper function to check valid range
def CheckForValidUReal(value, parameterName, objectName):
    if value < 0:
        raise ValueError("Error in "+objectName+": (float) parameter "+parameterName + " may not be negative, but received "+str(value))
        return 0.
    return value

#helper function to check valid range
def CheckForValidPReal(value, parameterName, objectName):
    if value <= 0:
        raise ValueError("Error in "+objectName+": (float) parameter "+parameterName + " must be positive (> 0), but received "+str(value))
        return 1. #this position is usually not reached
    return value

#helper: return True, if x is int, float, np.double, np.integer or similar types that can be automatically casted to pybind11
def IsValidNumber(x):
    if (isinstance(x, float) 
        or isinstance(x, int)
        or isinstance(x, np.double)
        or isinstance(x, np.integer)
        ):
        return True
    return False

#helper function to check valid range
def CheckForValidNumpyArray(value):
    if IsValidNumber(value): 
        return value
    else:
        return np.array(value)

'''


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def main(argv=None):
    parser = argparse.ArgumentParser(description='Emit python/exudyn/itemInterface.py from definitions/.')
    parser.add_argument('--output', default=os.path.join(im.repositoryRoot, 'python', 'exudyn',
                                                         'itemInterface.py'),
                        help='file to write (default: python/exudyn/itemInterface.py)')
    args = parser.parse_args(argv)

    text = EmitItemInterface(im.ItemDefinitions())
    with io.open(args.output, 'w', encoding='utf8') as file:
        file.write(text)
    return 0


if __name__ == '__main__':
    sys.exit(main())
