#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  Emits the documentation and stubs of the Python functions added to MainSystem
#           (@extends(exudyn.MainSystem)): MainSystemExt.rst and MainSystemCreateExt.rst in
#           tools/generators/generated/ (read by pybindEmitter.py, so this runs first),
#           docs/theDoc/MainSystemExt.tex, MainSystemCreateExt.tex and stubAutoBindingsExt.pyi
#           (read by createStubFiles.py). Moved out of utilitiesDocuGenerator.py (revision plan
#           step 33, part 2e).
#
# Usage:    python tools/generators/mainSystemExtensionDocsEmitter.py
#
# Author:   Johannes Gerstmayr
# Date:     2020-06-09 (created as utilitiesDocuGenerator.py), 2026-09-14 (emitter)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import io
import os
import sys

toolsDirectory = os.path.dirname(os.path.abspath(__file__))
if toolsDirectory not in sys.path:
    sys.path.insert(0, toolsDirectory)

from utilityDocsModel import *                                                   # noqa: E402,F403


def main():
    latexExtensions = {} #for C++ extension functions
    rstExtensions = {} #for C++ extension functions
    pyiExtensions = {} #for stub files of C++ extension functions
    #special strings, to put MainSystemExtensions (CreateMassPoint, ...) on top of RST and Latex description!
    latexExtensionsMainSystem = ''
    rstExtensionsMainSystem = ''

    for fileName in filesParsed:
        [functionList,classList,header] = ParsePythonFile(fileDir+fileName)
        moduleName, moduleNameLatex, moduleNamePython = ModuleNames(fileName)
        for funcDict in functionList:
            if 'belongsTo' not in funcDict:
                continue
            belongsTo = funcDict['belongsTo'].strip()
            functionDescription = funcDict['function']
            funcDict['functionDescriptionClean'] = functionDescription
            functionName = funcDict['functionName']

            sExamples = ''
            sExamplesRST = ''
            if addExampleReferences:
                exampleFunctionName = funcDict['functionName'].replace(belongsTo,'')
                [sExamples,sExamplesRST] = GenerateLatexStrKeywordExamples('UtilityFunction', exampleFunctionName, '', useLatex=False)

            belongsTo = funcDict['belongsTo'].strip()
            del funcDict['belongsTo']

            funcDict['function'] = functionDescription+' - NOTE that this function is added to MainSystem via Python function '+funcDict['functionName']+'.'
            funcDict['functionName'] = funcDict['functionName'].replace(belongsTo,'') 
            
            [sFuncLatex, sFuncRST, sPyi, sPy] = WriteFunctionDescription2LatexRST(funcDict, moduleNamePython, fileName, 
                                                                                  createPyiFile=True)
            
            if belongsTo not in latexExtensions:
                latexExtensions[belongsTo] = ''
                rstExtensions[belongsTo] = ''
                pyiExtensions[belongsTo] = ''
            
