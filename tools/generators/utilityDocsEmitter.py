#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  Emits the reference documentation of the utility modules from their docstrings:
#           docs/theDoc/pythonUtilitiesDescription.tex, docs/RST/pythonUtilities/*.rst and
#           docs/RST/confHelperPyUtilities.py. Moved out of utilitiesDocuGenerator.py (revision plan
#           revision2026 step R4.3, part 2e); re-pointed at Google-style docstrings via griffe by revision2026 steps R4.6 and R4.8.
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


def main():
    print('*****************************************')
    print('create documentation for exudyn utilities')
    listRST = [] #creates tuple of modulename and RST content
    sLatex = ''

    for fileName in filesParsed:
        # print('parse file:',fileName)
        sRST = ''
        [functionList,classList,header] = ParsePythonFile(fileDir+fileName)
        moduleName = fileName[:-3]
        moduleNameLatex = moduleName.replace('robotics/roboticsCore','robotics').replace('/','.')
    
        moduleNamePython = moduleName.split('/')[-1]
        baseModule = ''
        if '/' in moduleName:
            baseModule = moduleName.split('/')[0]
        
    
        strSub = ''
        sectionLevel = 2
        if '.' in moduleNameLatex: #don't do it for robotics core 
            strSub = 'sub'
            sectionLevel += 1
            # print('found / in ', moduleName)
            # print('  =>'+'\\my'+strSub+'subsection{Module: '+moduleNameLatex+'}\n')
        sLatex += '\\my'+strSub+'subsection{Module: '+moduleNameLatex+'}\n'
        sLatex += '\\label{sec:module:'+moduleNameLatex+'}\n'

        sRST += RSTlabelString('sec-module-'+moduleNameLatex.replace('.','-'))+'\n'
        sRST += RSTheaderString('Module: '+moduleNameLatex, sectionLevel)+'\n'
    
        if moduleNamePython != 'mainSystemExtensions': #no description for this!
            #*****************************************************
            if 'Details' in header: #write details as intro to section
                sLatex += header['Details'] #+ '\n'
                sRST += LatexString2RSTspecial(RemoveIndentation(header['Details']))
                #print('header=\n'+sRST)
            if len(header)>1:
                sLatex += '\\begin{itemize}[leftmargin=1.4cm]\n'
                sLatex += '\\setlength{\\itemindent}{-1.4cm}\n'
                sRST += '\n'
                for tag in headerTags:
                    if tag in header and tag != 'Details' and tag != 'Copyright':
                        if header[tag].find('\\') == -1:
                            sLatex += '\\item[]' + tag + ': ' + header[tag] #+ '\n'
                            sRST += '- '+ tag + ': ' + LatexString2RSTspecial(header[tag].replace('\n',' ')) + '\n'
                        else:
                            sRST += '- | ' + tag.strip() + ':'+'\n'
                            listString = header[tag].split('\\\\')
                            sLatex += '\\item[]' + tag + ':' + '\n' #+ listString[0] + ' \n'
                            sLatex += '\\vspace{-22pt}'
                            sLatex += '\\begin{itemize}[leftmargin=0.5cm]\n'
                            sLatex += '\\setlength{\\itemindent}{-0.5cm}\n'
                            for i in range(len(listString)-0):
                                sTag = listString[i+0]
                                sLatex += '\\item[]' + sTag.replace('\n',' ') + '\n'
                                sRST += '  | '+ LatexString2RSTspecial(sTag.replace('\n',' ').strip()) + '\n'
                            sLatex += '\\ei\n'
                        
                sLatex += '\\ei\n'
            sRST += '\n'
        else:
            mseText = 'NOTE: This module only contains links for extensions of C++ classes. The description is available in the respective descriptions of the C++ interface.\n'
            sLatex += mseText 
            sRST += mseText 

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

            if not isFirstFunction and moduleNamePython != 'mainSystemExtensions':# and not belongsTo:
                sLatex += "\\noindent\\rule{8cm}{0.75pt}\\vspace{1pt} \\\\ \n"
                sRST += "\n\n----\n\n" #horizontal ruler
                #sLatex += "\\hline\\vspace{3pt}\\\\ \n"
            
            #++++++++++++++++++++++++++++++++++++
            #add example references for function
            sExamples = ''
            sExamplesRST = ''
            if addExampleReferences:
                exampleFunctionName = funcDict['functionName'].replace(belongsTo,'')
                [sExamples,sExamplesRST] = GenerateLatexStrKeywordExamples('UtilityFunction', exampleFunctionName, '', useLatex=False)
        
            if belongsTo != '':
                #the MainSystem extension part is emitted by mainSystemExtensionDocsEmitter.py
                del funcDict['belongsTo']

            #add remaining part to original latex and RST
            if moduleNamePython != 'mainSystemExtensions': #no description for this!

                [sFuncLatex, sFuncRST, sPyi, sPy] = WriteFunctionDescription2LatexRST(funcDict, moduleNamePython, fileName, 
                                                                                      createPyiFile=False, 
                                                                                      redirectBelongsTo=(belongsTo != ''))

                sLatex += sFuncLatex
                sRST += sFuncRST

                if belongsTo:
                    textAdd = 'this function is directly available in MainSystem (mbs); it should be directly called as mbs.'+funcDict['functionName']+'(...).'
                    textAdd += ' For description of the interface, see the MainSystem Python extensions, '
                    mseLabel = 'sec:mainsystemextensions' + ':' + funcDict['functionName'] .replace('\\_','_')


                    textAddRST = textAdd + ' :ref:`'+Latex2RSTlabel(mseLabel)+'`\\ '+'\n'
                    textAdd += '\\refSection{'+mseLabel+'}.\n'
                
                    sRST += '\n'+'- | **NOTE**\\ : '+textAddRST + '\n'
                    sLatex += '\\bi\n  \\item \\mybold{NOTE}: ' + textAdd + '\n\\ei\n'

                if addExampleReferences and not belongsTo:
                    sLatex += sExamples
                    sRST += '\n'+sExamplesRST
        

            isFirstFunction=False



        #insert class descriptions with functions
        #there is no MainSystem extensions part here!
        for classDict in classList:
            SpecialAppend(localListClassNames, classDict['className'])
            # print(classDict['className'])
            #print(classDict)

        
            sLatex += '\\my'+strSub+'subsubsection{CLASS '+classDict['className']+' (in module '+moduleNameLatex+')}\n'
            #sLatex += '\\bi'
            sLatex += '\\noindent\\textcolor{steelblue}{{\\bf class description}}: ' + classDict['class']

            sRST += RSTlabelString('sec-module-'+moduleNameLatex.replace('.','-')+'-class-'+Latex2RSTlabel(classDict['className']))
            sRST += '\n' + RSTheaderString('CLASS '+classDict['className']+' (in module '+moduleNameLatex+')', level = 4)#sectionLevel)
            sRST += RSTmarkup('class description','**', False)+': ' + '\n\n' + \
                RemoveIndentation(LatexString2RSTspecial( classDict['class'] ), '    ') #+ '\n'


            #sLatex += '\\ei'
            localTags = docuTags.copy()
            localTags.remove('class')
            [sTags, sTagsRST] = DictToItemsText(classDict, localTags, '')
            if sTags != '':
                sLatex += '\\setlength{\\itemindent}{0.7cm}\n'
                sLatex += '\\begin{itemize}[leftmargin=0.7cm]\n'
                sLatex += sTags
                sLatex += '\\vspace{24pt}\\end{itemize}\n%\n'
                sRST += '\n' + sTagsRST + '\n'
            else:
                sLatex += '\\vspace{3pt} \\\\ \n' #add space for new class
                sRST += '\n'

            isFirstFunction = True
            for funcDict in classDict['functionList']:
                SpecialAppend(localListFunctionNames, funcDict['functionName'])
                
                if not isFirstFunction:
                    sLatex += "\\noindent\\rule{8cm}{0.75pt}\\vspace{1pt} \\\\ \n"
                    sRST += "\n----\n" #horizontal ruler

                    #sLatex += "\\hline\\vspace{3pt}\\\\ \n"
                [sFuncLatex, sFuncRST, sPyi, sPy] = WriteFunctionDescription2LatexRST(funcDict, 
                                                                                      moduleNamePython, 
                                                                                      fileName, 
                                                                                      isClassFunction=True, 
                                                                                      className=classDict['className'], 
                                                                                      createPyiFile=False)
                sLatex += sFuncLatex
                sRST += sFuncRST

                isFirstFunction=False

            #use split in class, for derived classes like InertiaCylinder(RigidBodyInertia)
            if addExampleReferences:
                [sExamples,sExamplesRST] = GenerateLatexStrKeywordExamples('UtilityFunction', classDict['className'].split('(')[0], '', useLatex=False)
                sLatex += sExamples
                sRST += '\n'+sExamplesRST


        sRST = sRST #.replace('**kwargs','\\*\\*kwargs').replace('*args','\\*args') #only needed, if not in literal
        #listRST += [(moduleNameLatex, LatexString2RSTspecial(sRST, replaceMarkups=False))]
        listRST += [(moduleNameLatex, sRST)]



    #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++        
    latexFile = theDocDir+'pythonUtilitiesDescription.tex'
    file=open(latexFile,'w',encoding='utf8')  #clear file by one write access
    file.write('% ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++')
    file.write('% description of python utility functions; generated by Johannes Gerstmayr')
    file.write('% ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n\n')
    file.write(sLatex)
    file.close()


    #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++        
    rstFile = rstDir+'pythonUtilities/pythonUtilities.rst'

    sRSTpreamble = RSTlabelString('sec-pythonUtilityFunctions')
    sRSTpreamble +="""
========================
Python Utility Functions
========================

This chapter describes in every subsection the functions and classes of the utility modules. 
These modules help to create multibody systems with the EXUDYN core module. Functions are implemented in Python and can be easily changed, extended and also verified by the user. **Check the source code** by entering these functions in Sypder and pressing ``CTRL + left mouse button``\\ . These Python functions are much slower than the functions available in the C++ core. Some matrix computations with larger matrices implemented in numpy and scipy, however, are parallelized and therefore very efficient.

Note that in general functions accept lists and numpy arrays. If not, an error will occur, which is easily tracked.
Furthermore, angles are generally provided in radian ($2\\pi$ equals $360\\,^o$) and no units are used for distances, but it is recommended to use SI units (m, kg, s) throughout.

Functions have been implemented, if not otherwise mentioned, by Johannes Gerstmayr.
"""
    sRSTpreamble = LatexString2RSTspecial(sRSTpreamble, replaceMarkups=False)


    if writeRST:
        file=io.open(rstFile,'w',encoding='utf8')  #clear file by one write access
        file.write(sRSTpreamble)
        file.close()

        sRSTindex = RSTheaderString('Python Utility Functions',0)
        sRSTindex += """
.. toctree::
   :maxdepth: 2
   
   pythonUtilities
"""

        for (name, text) in listRST:
            file=io.open(rstDir+'pythonUtilities/'+name+'.rst','w',encoding='utf8')  #clear file by one write access
            file.write(text)
            file.close()
            sRSTindex += '   '+name+'\n'

        file=io.open(rstDir+'pythonUtilities/index.rst','w',encoding='utf8')  #clear file by one write access
        file.write(sRSTindex)
        file.close()
    
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

    with open(rstDir+'confHelperPyUtilities.py', 'w',encoding='utf8') as f:
        f.write(sConfHelper)

    print('------- utilitiesDocu finished -----------')
    return 0


if __name__ == '__main__':
    sys.exit(main())
