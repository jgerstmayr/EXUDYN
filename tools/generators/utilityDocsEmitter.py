#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  Emits the reference documentation of the utility modules from their docstrings:
#           docs/theDoc/pythonUtilitiesDescription.tex, docs/RST/pythonUtilities/*.rst and
#           docs/RST/confHelperPyUtilities.py. Moved out of utilitiesDocuGenerator.py;
#           re-pointed at Google-style docstrings via griffe
#
# Usage:    python tools/generators/utilityDocsEmitter.py
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
from autoGenerateHelper import MarkdownLabel, MarkdownHeading, LatexText2Markdown, \
                               KeywordExamplesMarkdown                          # noqa: E402
from latexToMarkdown import NormalizeHeadings                                            # noqa: E402


def main():
    print('*****************************************')
    print('create documentation for exudyn utilities')
    listMarkdown = []   #(moduleName, Markdown)

    for fileName in filesParsed:
        sMarkdown = ''
        [functionList,classList,header] = ParsePythonFile(fileDir+fileName)
        moduleName = fileName[:-3]
        moduleNameDocu = moduleName.replace('robotics/roboticsCore','robotics').replace('/','.')
    
        moduleNamePython = moduleName.split('/')[-1]
        baseModule = ''
        if '/' in moduleName:
            baseModule = moduleName.split('/')[0]
        
    
        sMarkdown += MarkdownLabel('sec:module:'+moduleNameDocu)+'\n'
        sMarkdown += MarkdownHeading('Module: '+moduleNameDocu, 1)+'\n\n'
    
        if moduleNamePython != 'mainSystemExtensions': #no description for this!
            #*****************************************************
            if 'Details' in header: #write details as intro to section
                sMarkdown += LatexText2Markdown(RemoveIndentation(header['Details'])) + '\n\n'
            if len(header)>1:
                for tag in headerTags:
                    if tag in header and tag != 'Details' and tag != 'Copyright':
                        sMarkdown += ('- **' + tag + '**: '
                                      + LatexText2Markdown(header[tag]).replace('\n', ' ')
                                      + '\n')
        else:
            sMarkdown += ('NOTE: This module only contains links for extensions of C++ classes. '
                          'The description is available in the respective descriptions of the '
                          'C++ interface.\n\n')

        #*****************************************************
        cnt=0
        isFirstFunction = True
        #insert function descriptions 
        for funcDict in functionList:
            if 'functionName' not in funcDict:
                print('SpecialAppend: missing functionName in: ',funcDict)

            # if "example" in funcDict:#['defaultArgumentsList']:
                #print(funcDict)

            belongsTo = '' #for mainSystemExtensions
            if 'belongsTo' in funcDict:
                belongsTo = funcDict['belongsTo'].strip()

            SpecialAppend(localListFunctionNames, funcDict['functionName'].replace(belongsTo,''))

            functionDescription = funcDict['function']
            funcDict['functionDescriptionClean'] = functionDescription
            functionName = funcDict['functionName']

            exampleFunctionName = funcDict['functionName'].replace(belongsTo,'')
        
            if belongsTo != '':
                #the MainSystem extension part is emitted by mainSystemExtensionDocsEmitter.py
                del funcDict['belongsTo']

            #add remaining part to original latex and RST
            if moduleNamePython != 'mainSystemExtensions': #no description for this!
                sMarkdown += FunctionDescription2Markdown(funcDict, moduleNamePython, fileName,
                                                         headingLevel=2)
                if addExampleReferences and not belongsTo:
                    sMarkdown += KeywordExamplesMarkdown('UtilityFunction', exampleFunctionName)

                if belongsTo:
                    #the function is documented with the class it is added to; this note existed
                    #only in the LaTeX and RST branches
                    mseLabel = ('sec:mainsystemextensions:'
                                + funcDict['functionName'].replace(chr(92) + '_', '_'))
                    sMarkdown += ('- **NOTE**: this function is directly available in MainSystem '
                                  '(mbs); it should be directly called as mbs.'
                                  + funcDict['functionName'].replace(chr(92) + '_', '_')
                                  + '(...). For description of the interface, see the MainSystem '
                                  'Python extensions, {ref}`'
                                  + MarkdownLabelName(mseLabel) + '`\n\n')
        

            isFirstFunction=False



        #insert class descriptions with functions
        #there is no MainSystem extensions part here!
        for classDict in classList:
            SpecialAppend(localListClassNames, classDict['className'])
            # print(classDict['className'])
            #print(classDict)

        
            sMarkdown += ('\n' + MarkdownLabel('sec:module:' + moduleNameDocu + ':class:'
                                              + classDict['className']) + '\n'
                          + MarkdownHeading('CLASS ' + classDict['className'] + ' (in module '
                                            + moduleNameDocu + ')', 2) + '\n\n'
                          + '**class description**: '
                          + LatexText2Markdown(classDict['class']).replace('\n', ' ') + '\n\n')

            localTags = docuTags.copy()
            localTags.remove('class')
            sMarkdown += Tags2Markdown(classDict, localTags)

            for funcDict in classDict['functionList']:
                SpecialAppend(localListFunctionNames, funcDict['functionName'])
                sMarkdown += FunctionDescription2Markdown(funcDict, moduleNamePython, fileName,
                                                         isClassFunction=True,
                                                         className=classDict['className'],
                                                         headingLevel=3)

            #use split in class, for derived classes like InertiaCylinder(RigidBodyInertia)
            if addExampleReferences:
                sMarkdown += KeywordExamplesMarkdown('UtilityFunction',
                                                     classDict['className'].split('(')[0])

        listMarkdown += [(moduleNameDocu, sMarkdown)]



    #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #MARKDOWN This emitter wrote
    #docs/theDoc/pythonUtilitiesDescription.tex and docs/RST/pythonUtilities/*.rst until
    #2026-09-20; the pages are in docs/generated/, where emitter output belongs (D10).
    markdownDir = os.path.join(paths.repositoryRoot, 'docs', 'generated', 'pythonUtilities')
    os.makedirs(markdownDir, exist_ok=True)

    def Banner(title=''):
        #the page's own H1 comes from the module heading the loop wrote; the index has no such
        #heading, so it passes a title
        banner = ('<!-- GENERATED by tools/generators/utilityDocsEmitter.py from the docstrings '
                  'of python/exudyn - do not edit -->\n')
        return banner + ('# ' + title + '\n\n' if title != '' else '\n')

    #the chapter label, which the manual references as sec:pythonUtilityFunctions
    indexText = Banner()
    indexText += MarkdownLabel('sec:pythonUtilityFunctions') + chr(10)
    indexText += MarkdownHeading('Python Utility Functions', 0) + chr(10) + chr(10)
    indexText += '\n'.join([
        'This chapter describes in every section the functions and classes of the utility modules.',
        'These modules help to create multibody systems with the Exudyn core module. Functions are',
        'implemented in Python and can be changed, extended and verified by the user - **check the',
        'source code** by entering a function in Spyder and pressing **CTRL + left mouse button**.',
        'These Python functions are much slower than the functions of the C++ core; some matrix',
        'computations with larger matrices, implemented in numpy and scipy, are parallelised and',
        'therefore very efficient.',
        '',
        'Note that in general functions accept lists and numpy arrays. If not, an error will occur,',
        'which is easily tracked. Furthermore, angles are generally provided in radian ($2\\pi$ equals',
        '$360\\,^o$) and no units are used for distances, but it is recommended to use SI units',
        '(m, kg, s) throughout.',
        '',
        'Functions have been implemented, if not otherwise mentioned, by Johannes Gerstmayr.',
        ])
    indexText += '\n\n```{toctree}\n:maxdepth: 2\n\n'

    for (name, markdown) in listMarkdown:
        page = NormalizeHeadings(Banner() + markdown.strip()) + '\n'
        with io.open(os.path.join(markdownDir, name + '.md'), 'w', encoding='utf8',
                     newline='\n') as file:
            file.write(page)
        indexText += name + '\n'

    with io.open(os.path.join(markdownDir, 'utilitiesIndex.md'), 'w', encoding='utf8',
                 newline='\n') as file:
        file.write(indexText + '```\n')

    print('utilityDocsEmitter: ' + str(len(listMarkdown) + 1) + ' Markdown file(s) written')

    #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++        
    #export data for conf.py
    #write class names for confHelperPyUtilities.py

    sConfHelper = ''
    sConfHelper += '#this is a helper file to define additional keywords for examples\n'
    sConfHelper += '#Created: 2023-03-17, Johannes Gerstmayr\n\n'

    #list of classes and function names:
    sConfHelper += 'listPyFunctionNames=['
    for s in localListFunctionNames:
        sConfHelper += "'" + s + "'" + ', '
    sConfHelper += ']\n\n'

    sConfHelper += 'listPyClassNames=['
    for s in localListClassNames:
        s = s.split('(')[0] #KirchhoffMaterial(MaterialBaseClass), InverseKinematicsNumerical()
        sConfHelper += "'" + s + "'" + ', '
    sConfHelper += ']\n\n'

    with open(paths.generatedDir+'confHelperPyUtilities.py', 'w',encoding='utf8') as f:
        f.write(sConfHelper)

    print('------- utilitiesDocu finished -----------')
    return 0


if __name__ == '__main__':
    sys.exit(main())
