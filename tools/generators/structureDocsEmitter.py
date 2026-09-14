#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  Emits the structure reference documentation from definitions/: docs/theDoc/interfaces.tex
#           and docs/RST/structures/*.rst with StructuresAndSettingsIndex.rst (revision plan step 33,
#           part 2c). Moved out of src/pythonGenerator/pythonAutoGenerateSystemStructures.py; the
#           output is byte-identical. Step 50 replaces the LaTeX/RST pipeline as a whole.
#
# Usage:    python tools/generators/structureDocsEmitter.py
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
#the shared text helpers still live with the old generators until step 33 part 2g moves them
generatorDirectory = os.path.normpath(os.path.join(toolsDirectory, '..', '..', 'src', 'pythonGenerator'))
if generatorDirectory not in sys.path:
    sys.path.insert(0, generatorDirectory)

import io                                                               # noqa: E402

from structureModel import *                                            # noqa: E402,F403


#************************************************
#the documentation of one structure
def StructureDocs(parseInfo, parameterList):
    """returns [LaTeX text, RST text, parameter changes list]"""
    plr = PyLatexRST()
    plr.AddDocu(parseInfo['latexText']) #.replace('\\n','\n') #this is the string for latex documentation
    
    parameterListSorted = SortedParameters(parameterList)
    hasPybindInterface = HasPybindInterface(parseInfo, parameterList)
    typicalPaths = TypicalPaths(parseInfo) if hasPybindInterface else []

    if hasPybindInterface: #otherwise do not include the description into latex doc

        #print('typical:',typicalPaths)

        descriptionStr = parseInfo['classDescription']
        if not descriptionStr.endswith('.'): 
            descriptionStr += '. '
        
        plr.sLatex += '\n%+++++++++++++++++++++++++++++++++++\n'
        plr.AddDocu(Str2Latex(descriptionStr, replaceCurlyBracket=False)+
                    '\n\n\\noindent '+
                    parseInfo['class'] + ' has the following items:\n', 
                    section=parseInfo['class'], sectionLevel=3, 
                    sectionLabel='sec:' + parseInfo['class'].replace(' ',''))
        plr.sRST += '\n' #newline for start of list

        # plr.sLatex += '\mysubsubsection{' + parseInfo['class'] + '} \label{sec:' + parseInfo['class'].replace(' ','') + '}\n'
        # plr.sLatex += Str2Latex(descriptionStr, replaceCurlyBracket=False) + '\\\\ \n'
        # plr.sLatex += '%\n\\noindent '
        # plr.sLatex += parseInfo['class'] + ' has the following items:\n'
        plr.sLatex += '%reference manual TABLE\n'
        plr.sLatex += '\\begin{center}\n'
        plr.sLatex += '  \\footnotesize\n'
        plr.sLatex += '  \\begin{longtable}{| p{4.2cm} | p{2.5cm} | p{0.3cm} | p{3.0cm} | p{6cm} |}\n'
        plr.sLatex += '    \\hline\n'
        plr.sLatex += '    \\bf Name & \\bf type / function return type & \\bf size & \\bf default value / function args & \\bf description \\\\ \\hline\n'
    
        for parameter in parameterListSorted:
            if IsDeprecatedParameter(parameter):
                continue
            if (parameter['lineType'].find('V') != -1 and 
                parameter['cFlags'].find('P') != -1 and
                parameter['type'].find('ResizableVector') == -1): #only if it is a member variable
                
                sString = ''
                if (parameter['type'] == 'String' or parameter['type'] == 'FileName'):
                    sString="'"
                #write latex doc:
                defaultValueStr = parameter['defaultValue']
                paramDescriptionStr = parameter['parameterDescription'].replace('_','\\_')
                if len(defaultValueStr) > 18:
                    paramDescriptionStr = '\\tabnewline ' + paramDescriptionStr
                pythonName = Str2Latex(parameter['pythonName']) 
                typeName = Str2Latex(parameter['type'])
                
                # if len(pythonName)>28:  #inside plr.SystemStructuresWriteDefRow
                #     typeName = '\\tabnewline ' + typeName
                    
                if parameter['type'] != 'String' and parameter['type'] != 'FileName': #don't do this for file names, because 'f' is erased!
                    defaultValueStr = Str2Latex(defaultValueStr, True)

                plr.SystemStructuresWriteDefRow(pythonName, typeName, Str2Latex(parameter['size']), 
                                            sString+defaultValueStr+sString, paramDescriptionStr, 
                                            typicalPaths=typicalPaths, isFunction=False)
                                

            if (parameter['lineType'].find('F') != -1) and (parameter['cFlags'].find('P') != -1): #only if it is a function
                #write latex doc:
                functionName = Str2Latex(parameter['pythonName'])
                argStr = parameter['args']
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

                functionType = Str2Latex(parameter['type'])
                # if (len(functionName)>28):  #done now in SystemStructuresWriteDefRow
                #     functionType = '\\tabnewline ' + functionType

                # plr.sLatex += '    ' + functionName + ' & '
                # plr.sLatex += '    ' + functionType + ' & '
                # plr.sLatex += '    ' + Str2Latex(parameter['size']) + ' & '

                # plr.sLatex += '    ' + argStr + ' & '
                # plr.sLatex += '    ' + Str2Latex(parameter['parameterDescription'], replaceCurlyBracket=False) + '\\\\ \\hline\n' #Str2Latex not used, must be latex compatible!!!

                plr.SystemStructuresWriteDefRow(functionName, functionType, Str2Latex(parameter['size']), argStr, 
                                            Str2Latex(parameter['parameterDescription'], replaceCurlyBracket=False), isFunction=True)

                
        plr.sLatex += '	  \\end{longtable}\n'
        plr.sLatex += '	\\end{center}\n'



    parameterChangesList = ParameterChangesList(parseInfo, parameterListSorted, typicalPaths)
    return [plr.sLatex, plr.sRST, parameterChangesList]


def main():
    rstFileDict={'SimulationSettings':'',
                 'VisualizationSettings':'',
                 'CSolverStructures':'',
                 'MainSolver':'',
                 'PyStructuralElementsDataStructures':'',
                 'BeamSectionGeometry':'',
                 } #contains available file names and text

    globalLatexStr = '' #this is the whole string for the latex docu
    globalParameterChangesList = []

    for parseInfo, parameterList in LegacyStructures():
        [latexStr, rstStr, parameterChangesList] = StructureDocs(parseInfo, parameterList)
        globalLatexStr += latexStr
        globalParameterChangesList += parameterChangesList

        #the changes of the visualization substructures are listed at VisualizationSettings
        if not HasTopClass(parseInfo['class']) and len(globalParameterChangesList) != 0:
            (globalLatexStr, rstStr) = ParameterChanges2LatexRST(globalParameterChangesList, globalLatexStr, rstStr)
            globalParameterChangesList = []

        fileName = parseInfo['writeFile'].split('.')[0]
        if fileName in rstFileDict:
           rstFileDict[fileName] += rstStr

    latexText = """
This section includes the reference manual for structures (such as for solvers, helper structures, etc.) 
and settings which are available in the python interface, e.g., simulation settings, visualization settings. 
The data is auto-generated from the according interfaces in order to keep fully up-to-date with changes.
"""
    globalLatexStr += latexText
    
    fileLatex=open(paths.theDocDir+'interfaces.tex','w',encoding='utf8')  #clear file by one write access
    fileLatex.write('% definition of structures\n')
    fileLatex.write(globalLatexStr)
    fileLatex.close()

    rstDir = paths.rstDir+'structures/'
    rstIndex = """
=======================
Structures and Settings
=======================
"""
    rstIndex += latexText
    rstIndex += """
.. toctree::
   :maxdepth: 2

"""
    
    
    for key, value in rstFileDict.items():
        # print('RST: write '+key)#, ':',value[:200])
        file=io.open(rstDir+key+'.rst','w',encoding='utf8')  #clear file by one write access
        file.write(value+'\n')
        file.close()
        rstIndex += '   '+key+'\n'

    file=io.open(rstDir+'StructuresAndSettingsIndex.rst','w',encoding='utf8')  #clear file by one write access
    file.write(rstIndex+'\n')
    file.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
