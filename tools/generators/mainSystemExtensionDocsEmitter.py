#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  Emits the documentation and stubs of the Python functions added to MainSystem
#           (@extends(exudyn.MainSystem)): MainSystemExt.md and MainSystemCreateExt.md in
#           tools/generators/generated/ (read by pybindEmitter.py, so this runs first), and
#           stubAutoBindingsExt.pyi (read by createStubFiles.py). Moved out of
#           utilitiesDocuGenerator.py; the .tex
#           files went and the .rst ones in R7.1.7.
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
    markdownExtensions = {}  #per class the functions are added to; spliced by pybindEmitter
    pyiExtensions = {}       #for the stub files of the extension functions
    #the Create* functions go into their own fragment, so that the reference manual can put them
    #in front of everything else
    markdownExtensionsMainSystem = ''

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

            exampleFunctionName = funcDict['functionName'].replace(belongsTo,'')

            belongsTo = funcDict['belongsTo'].strip()
            del funcDict['belongsTo']

            funcDict['function'] = functionDescription+' - NOTE that this function is added to MainSystem via Python function '+funcDict['functionName']+'.'
            funcDict['functionName'] = funcDict['functionName'].replace(belongsTo,'') 
            
            sPyi = FunctionStub(funcDict)
            
            #written from the parsed dictionary; the heading is one
            #level below the class section that pybindEmitter puts it into
            markdownTemp = FunctionDescription2Markdown(funcDict, moduleNamePython, fileName,
                                                        headingLevel=3,
                                                        labelModule='mainsystemextensions')

            if belongsTo not in markdownExtensions:
                markdownExtensions[belongsTo] = ''
                pyiExtensions[belongsTo] = ''

            pyiExtensions[belongsTo] += sPyi

            if addExampleReferences:
                markdownTemp += KeywordExamplesMarkdown('UtilityFunction', exampleFunctionName)

            #the Create* functions of mainSystemExtensions.py go into their OWN pair of files, so
            #that the reference manual can put them in front of everything else; the rest is
            #collected per class that it is added to
            if moduleNamePython != 'mainSystemExtensions':
                markdownExtensions[belongsTo] += markdownTemp
            else:
                markdownExtensionsMainSystem += markdownTemp

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
    for key in markdownExtensions:
        Write(paths.generatedDir + key + 'Ext.md', markdownExtensions[key])
        #ONE stub file for all keys: the original opened it with 'w' inside the loop, so only the
        #last class would have survived, and then wrote it a second time with the same content.
        #There is one key today, which is why nobody ever saw either half of that
        stubContent += '\nclass ' + key + ':\n' + pyiExtensions[key]

    Write(paths.generatedDir + 'stubAutoBindingsExt.pyi', stubContent)
    Write(paths.generatedDir + 'MainSystemCreateExt.md', markdownExtensionsMainSystem)

    #NOT written, deliberately: the original wrote python/exudyn/mainSystemExtensions.py from the
    #'exu.MainSystem.X = ...' lines it collected - overwriting a hand-written module that this
    #generator PARSES. It is not among the declared outputs of the stage and it is not restored.

    print('mainSystemExtensionDocsEmitter: ' + str(len(written)) + ' file(s) written')


if __name__ == '__main__':
    main()
