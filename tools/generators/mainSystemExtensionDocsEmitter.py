#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  Emits the documentation and stubs of the Python functions added to MainSystem
#           (@extends(exudyn.MainSystem)): MainSystemExt.rst and MainSystemCreateExt.rst in
#           tools/generators/generated/ (read by pybindEmitter.py, so this runs first),
#           docs/theDoc/MainSystemExt.tex, MainSystemCreateExt.tex and stubAutoBindingsExt.pyi
#           (read by createStubFiles.py). Moved out of utilitiesDocuGenerator.py (revision plan
#           revision2026 step R4.3, part 2e).
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

            latexTemp = sFuncLatex + '\n'
            rstTemp = '\n' + sFuncRST
            pyiExtensions[belongsTo] += sPyi

            if addExampleReferences:
                latexTemp += sExamples
                rstTemp += '\n' + sExamplesRST

            #the Create* functions of mainSystemExtensions.py go into their OWN pair of files, so
            #that the reference manual can put them in front of everything else; the rest is
            #collected per class that it is added to
            if moduleNamePython != 'mainSystemExtensions':
                latexExtensions[belongsTo] += latexTemp
                rstExtensions[belongsTo] += rstTemp
            else:
                latexExtensionsMainSystem += latexTemp
                rstExtensionsMainSystem += rstTemp

    #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #WRITE, which is what this file stopped doing between 2026-09-14 and 2026-09-19 (#2526). The
    #split of step R4.3 part 2e carried the loop across and not the writes, so the generator ran,
    #exited 0 and produced nothing - and the regeneration gate could not see it, because a
    #generator that writes nothing always agrees with the commit.
    written = []

    def Write(fileName, content):
        with io.open(fileName, 'w', encoding='utf8') as file:
            file.write(content)
        written.append(fileName)

    stubContent = ''
    for key in latexExtensions:
        Write(paths.generatedDir + key + 'Ext.rst', rstExtensions[key])
        Write(theDocDir + '/' + key + 'Ext.tex', latexExtensions[key])
        #ONE stub file for all keys: the original opened it with 'w' inside the loop, so only the
        #last class would have survived, and then wrote it a second time with the same content.
        #There is one key today, which is why nobody ever saw either half of that
        stubContent += '\nclass ' + key + ':\n' + pyiExtensions[key]

    Write(paths.generatedDir + 'stubAutoBindingsExt.pyi', stubContent)
    Write(paths.generatedDir + 'MainSystemCreateExt.rst', rstExtensionsMainSystem)
    Write(theDocDir + '/MainSystemCreateExt.tex', latexExtensionsMainSystem)

    #NOT written, deliberately: the original wrote python/exudyn/mainSystemExtensions.py from the
    #'exu.MainSystem.X = ...' lines it collected - overwriting a hand-written module that this
    #generator PARSES. It is not among the declared outputs of the stage and it is not restored.

    print('mainSystemExtensionDocsEmitter: ' + str(len(written)) + ' file(s) written')


if __name__ == '__main__':
    main()
