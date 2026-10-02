#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  Emits python/exudyn/itemInterface.py - the Python classes (ObjectMassPoint,
#           VObjectMassPoint, ...) that users instantiate to build item dictionaries - directly
#           from definitions/. The first emitter split out of pythonAutoGenerateObjects.py
#           ; the code was moved from there, so the output is
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
from userFunctionModel import ReadUserFunction, CheckAgainstCpp, PythonType          # noqa: E402
from itemModel import (pyFunctionTypeConversion, IsAVector,                         # noqa: E402
                       IsASimpleMatrix, IsAArrayIndex, IsTypeWithRangeCheck, ExtractMathSymbol,
                       possibleTypes)
from autoGenerateHelper import (GetTypesStringDocu,                                # noqa: E402
                                SplitSummaryDescription, GoogleDocstringRenderer,
                                CleanStringForPyiDescription)

ADD_DOCSTRINGS = True
space4 = '    '


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def ProtocolName(className, pythonName):
    """the name of the Protocol of one user function: ObjectGroundGraphicsDataUserFunction"""
    return className + pythonName[0].upper() + pythonName[1:]


def ProtocolText(className, pythonName, userFunction):
    """the Protocol of one user function: what an editor completes and checks a function against

    It costs nothing at runtime - a Protocol is never instantiated and nothing inherits from it - and
    it is the part of #2664 a user feels: with the parameter annotated, an editor completes the
    arguments of the function that is being written and marks a wrong one (#2664)."""
    arguments = [(name, PythonType(annotation)) for (name, annotation) in userFunction.arguments]
    docstring = {'kind': 'class',
                 'summary': CleanStringForPyiDescription(userFunction.summary),
                 'description': CleanStringForPyiDescription(userFunction.details),
                 'inputs': [{'name': name,
                             'type_hint': pythonType,
                             'description': CleanStringForPyiDescription(
                                 userFunction.argumentText.get(name, ''))}
                            for ((name, _), (_, pythonType))
                            in zip(userFunction.arguments, arguments)],
                 'output': {'type_hint': PythonType(userFunction.returnType),
                            'description': CleanStringForPyiDescription(userFunction.returnText)},
                 'notes': [], 'examples': [], 'author': None, 'date': None, 'belongs_to': None}

    s = 'class ' + ProtocolName(className, pythonName) + '(Protocol):\n'
    s += GoogleDocstringRenderer().render(docstring, indent=space4) + '\n'
    s += space4 + 'def __call__(self'
    for (name, pythonType) in arguments:
        s += ', ' + name + ': ' + pythonType
    s += ') -> ' + PythonType(userFunction.returnType) + ': ...\n\n'
    return s


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
        protocolName = None

        cnt = 0
        for arg in fcnArgs[1:]: #omit MainSystem
            fcnArgsList += ['arg'+str(cnt)]
            fcnTypesList += [arg.strip()]
            cnt+=1

        #a parameter that carries a Python def knows what its arguments are CALLED; arg0, arg1, ...
        #are the fallback for the ones still written as prose (#2664).
        #The def is checked against the std::function here, because this is the one place that holds
        #both; tools/checkDefinitions.py reports the same findings with the file and the line
        if item.get('userFunction') is not None:
            userFunction = ReadUserFunction(item['userFunction'], userFunctionName)
            findings = CheckAgainstCpp(userFunction, item['stdFunctionType'])
            if len(findings) != 0:
                raise ValueError(classType + itemType + '.' + userFunctionName + ': the Python def '
                                 + '; '.join(findings))
            fcnArgsList = [name for (name, _) in userFunction.arguments]
            protocolName = ProtocolName(classType + itemType, userFunctionName)

        entry = [fcnTypesList, fcnArgsList, [fcnType]]
        if protocolName is not None:
            #the Protocol an editor checks against; advancedUtilities names it when a user function
            #has the wrong number of arguments (#2664)
            entry.append([protocolName])
        userFunctionArgsDict[classType+itemType+','+userFunctionName] = entry

    return userFunctionArgsDict


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def ItemDocstrings(definition):
    """the Google-style docstring data of the item class and of its visualization class"""
    className = definition['className']
    classType = definition.get('classType', '')
    (pyiSummary, pyiDescription) = SplitSummaryDescription(CleanStringForPyiDescription(
        im.OverallDescription(definition)))
    dataDocstring = {'kind': 'classFunction', 'notes': [], 'inputs': [], 'argTypes': False}
    dataDocstring['summary'] = pyiSummary
    dataDocstring['description'] = pyiDescription
    dataDocstringV = {'kind': 'classFunction',
                      'summary': 'Visualization data for ' + className,
                      'inputs': [], 'argTypes': False}

    requestedMarkerString = ''
    itemTypeString = ''
    requestedNodeString = ''
    for member in definition['members']:
        if im.IsInterfaceParameter(member) and not im.IsReadOnly(member): #the __init__ arguments
            [parameterDescription, mathSymbol] = ExtractMathSymbol(im.Description(member))
            thisDataDocString = dataDocstringV if 'V' in im.Destination(member) else dataDocstring
            typeHint = tm.Render(im.TypeName(member), 'pyTyping', 'items')
            description = CleanStringForPyiDescription(parameterDescription).strip()
            if typeHint and typeHint not in description: #the type stays readable; Args have no '(type)'
                description += ('' if description == '' else ';') + ' type: ' + typeHint
            thisDataDocString['inputs'].append({'name': member['pythonName'],
                                                'description': description.strip()})
        elif member['pythonName'] == 'GetRequestedMarkerType':
            requestedMarkerString = GetTypesStringDocu(im.DefaultValueString(member), 'Marker',
                                                        possibleTypes['Marker'], ' +')
        elif member['pythonName'] == 'GetRequestedNodeType':
            requestedNodeString = GetTypesStringDocu(im.DefaultValueString(member), 'Node',
                                                      possibleTypes['Node'], ' +')
        elif member['pythonName'] == 'GetType':
            searchType = classType
            if classType == 'Object': searchType += 'Type'
            itemTypeString = GetTypesStringDocu(im.DefaultValueString(member), searchType,
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

    dataDocstring['inputs'].append({'name': 'visualization',
                                    'description': 'visualization data, see V' + className})
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

            defaultValueStr = sString+im.DefaultValuePython(member)+sString

            #special treatment of BodyGraphicsData
            if typeName == 'BodyGraphicsData' or typeName == 'BodyGraphicsDataList':
                defaultValueStr = '[]'

            #write item interface class initialization, constructor and iterator doc:
            tempVPythonDict = "'" + pythonName + "': "
            tempPythonClass = ', ' + pythonName
            #a parameter that is a user function is annotated with its Protocol, which is what makes
            #an editor complete the function being written. The 0 is in the annotation because it is
            #the value that means "no user function" (#2664)
            if member.get('userFunction') is not None:
                tempPythonClass += (': Union[' + ProtocolName(className, pythonName) + ', int]')
            if len(defaultValueStr) != 0:
                tempPythonClass += ' = ' + defaultValueStr
                tempVPythonDict += defaultValueStr
            else:
                tempVPythonDict += "None"

            #range checks are done in C++ on every write path
            parameterWithCheck = pythonName
            if (IsAVector(typeName)
                or IsASimpleMatrix(typeName)
                ):
                if typeName == 'NumpyVector':
                    parameterWithCheck = 'CheckForValidNumpyArray('+parameterWithCheck+')'
                elif typeName == 'NumpyMatrix':
                    parameterWithCheck = 'CheckForValidNumpyArray('+parameterWithCheck+')'
                elif defaultValueStr == 'None': #None means not given, e.g. the parts of an HT (#2793)
                    parameterWithCheck = 'None if ' + pythonName + ' is None else np.array(' + pythonName + ')'
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

    #one Protocol per user function that is written as a Python def, before the classes that use it
    #(#2664)
    for definition in definitions:
        for member in definition['members']:
            if member.get('userFunction') is not None:
                s += ProtocolText(definition['className'], member['pythonName'],
                                  ReadUserFunction(member['userFunction'], member['pythonName']))

    for classType in im.itemTypeOrder:
        s += '#+++++++++++++++++++++++++++++++\n#' + classType.upper() + '\n'
        for definition in definitions:
            if definition.get('classType', '') == classType:
                s += ItemClasses(definition)

    #__all__ after the imports, from the same rule tools/checkAll.py checks
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
import copy
from typing import Protocol, Union \n

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
