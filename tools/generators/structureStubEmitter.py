#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  Emits tools/generators/generated/stubSystemStructures.pyi, the stub fragment of the
#           structures (SimulationSettings, VisualizationSettings, ...) that createStubFiles.py
#           assembles, from definitions/ (revision2026 step R4.3, part 2c). Moved out of
#           src/pythonGenerator/pythonAutoGenerateSystemStructures.py; the output is byte-identical.
#
# Usage:    python tools/generators/structureStubEmitter.py
#
# Author:   Johannes Gerstmayr
# Date:     2018-05-18 (created as pythonAutoGenerateSystemStructures.py), 2026-09-14 (emitter)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import os
import sys

toolsDirectory = os.path.dirname(os.path.abspath(__file__))
if toolsDirectory not in sys.path:
    sys.path.insert(0, toolsDirectory)

from structureModel import *                                            # noqa: E402,F403
import typeModel as tm                                                  # noqa: E402


def StructureStub(parseInfo):
    """the stub text of one structure definition; empty if it has no Python interface"""
    parameterList = parseInfo['members']
    stubStr = '' #string for .pyi file
    spaces4 = '    '
    if not HasPybindInterface(parseInfo, parameterList):
        return stubStr
    pythonClass = PythonClassName(parseInfo)

    stubStr += '\n#information for '+ pythonClass + '\n'
    stubStr += 'class ' + pythonClass + ':\n'
    if ADD_DOCSTRINGS: 
        stubStr += DocStringGoogleFromPlainText(Header(parseInfo, 'classDescription'),
                                                addSpaces=' '*4, multiline=True)

    for parameter in SortedParameters(parameterList):
        if IsDeprecatedParameter(parameter):
            continue
        if (IsVariable(parameter) and
            HasFlag(parameter, 'P') and
            parameter['type'].find('ResizableVector') == -1): #only if it is a member variable
            stubStr += spaces4+parameter['pythonName']+': '
            stubStr += tm.Render(parameter['type'], 'stub', 'structures') + '\n'
            if ADD_DOCSTRINGS: 
                stubStr += DocStringGoogleFromPlainText(text=ParameterDescription2DocString(Description(parameter)),
                                                        addSpaces=' '*4, multiline=False)

        if (IsFunction(parameter)) and (HasFlag(parameter, 'P')): #only if it is a function
            functionName = Str2Latex(parameter['pythonName'])
            argStr = Args(parameter)
            if (argStr != ''):
                #functionName += '(...)' #now added in SystemStructuresWriteDefRow
                argSplit = argStr.split(',') #split into list of args
                argStr = ''
                argSep = '' #no comma for first time
                for item in argSplit:
                    argName = item.split(' ')[-1] #last word in args is the name of the argument, e.g. in const MainSystem& mainSystem ==> mainSystem
                    argName = Str2Latex(argName)
                    argStr += argSep + argName.replace('=true','=True').replace('=false','=False')
                    argSep = ', '
            stubStr += spaces4+'@overload\n'
            stubStr += spaces4+'def '+functionName+'('+argStr.replace('\\_','_')+')'+' -> '+tm.Render(parameter['type'], 'stub', 'structures')+': ...\n'
    return stubStr


def main():
    globalStubStr = ''
    for parseInfo in StructureDefinitions():
        globalStubStr += StructureStub(parseInfo)

    globalStubStr = """
#This is the stub file for system structures, such as SimulationSettings and VisualizationSettings
#This file will greatly improve autocompletion

""" + globalStubStr

    fileStub=open(paths.generatedDir+'stubSystemStructures.pyi','w',encoding='utf8')  #clear file by one write access
    fileStub.write(globalStubStr)
    fileStub.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
