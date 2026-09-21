#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  Emits the structure reference documentation from definitions/: docs/theDoc/interfaces.tex
#           and docs/RST/structures/*.rst with StructuresAndSettingsIndex.rst (revision2026 step R4.3,
#           part 2c). Moved out of src/pythonGenerator/pythonAutoGenerateSystemStructures.py; the
#           output is byte-identical. revision2026 step R7.1 replaces the LaTeX/RST pipeline as a whole.
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

import io                                                               # noqa: E402

from structureModel import *                                            # noqa: E402,F403
from autoGenerateHelper import LatexText2Markdown                      # noqa: E402
from latexToMarkdown import NormalizeHeadings                                    # noqa: E402


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

        #the same table in Markdown (revision2026 step R7.1.6)
        plr.sMarkdown += ('\n| Name | type / function return type | size | default value / function '
                          'args | description |\n|---|---|---|---|---|\n')
    
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
        plr.sMarkdown += '\n'



    parameterChangesList = ParameterChangesList(parseInfo, parameterListSorted, typicalPaths)
    return [plr.sLatex, plr.sRST, plr.sMarkdown, parameterChangesList]


def main():
    rstFileDict={'SimulationSettings':'',
                 'VisualizationSettings':'',
                 'CSolverStructures':'',
                 'MainSolver':'',
                 'PyStructuralElementsDataStructures':'',
                 'BeamSectionGeometry':'',
                 } #contains available file names and text

    markdownFileDict = {key: '' for key in rstFileDict}   #revision2026 step R7.1.6

    globalLatexStr = '' #this is the whole string for the latex docu
    globalParameterChangesList = []

    for parseInfo, parameterList in LegacyStructures():
        [latexStr, rstStr, markdownStr, parameterChangesList] = StructureDocs(parseInfo,
                                                                              parameterList)
        globalLatexStr += latexStr
        globalParameterChangesList += parameterChangesList

        #the changes of the visualization substructures are listed at VisualizationSettings
        if not HasTopClass(parseInfo['class']) and len(globalParameterChangesList) != 0:
            (globalLatexStr, rstStr) = ParameterChanges2LatexRST(globalParameterChangesList, globalLatexStr, rstStr)
            markdownStr += ParameterChanges2Markdown(globalParameterChangesList)
            globalParameterChangesList = []

        fileName = parseInfo['writeFile'].split('.')[0]
        if fileName in rstFileDict:
           rstFileDict[fileName] += rstStr
           markdownFileDict[fileName] += markdownStr

    latexText = """
This section includes the reference manual for structures (such as for solvers, helper structures, etc.) 
and settings which are available in the python interface, e.g., simulation settings, visualization settings. 
The data is auto-generated from the according interfaces in order to keep fully up-to-date with changes.
"""
    globalLatexStr += latexText

    #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #MARKDOWN, revision2026 step R7.1.6. This emitter wrote docs/theDoc/interfaces.tex and
    #docs/RST/structures/*.rst until 2026-09-20; both are gone. The pages live in
    #docs/generated/, which is where emitter output belongs (decision D10), and every file says
    #in its first line that it is generated.
    markdownDir = os.path.join(paths.repositoryRoot, 'docs', 'generated', 'structures')
    os.makedirs(markdownDir, exist_ok=True)

    def Banner(title):
        return ('<!-- GENERATED by tools/generators/structureDocsEmitter.py from definitions/ '
                '- do not edit -->\n'
                '# ' + title + '\n\n')

    written = 0
    indexText = Banner('Structures and Settings')
    indexText += LatexText2Markdown(latexText).strip() + '\n\n'
    indexText += '```{toctree}\n:maxdepth: 2\n\n'

    for key, value in markdownFileDict.items():
        #the LaTeX section depths are relative to the whole document, so a page would jump from
        #its own H1 to H3 - which Sphinx refuses under -W
        page = NormalizeHeadings(Banner(key) + value.strip()) + '\n'
        with io.open(os.path.join(markdownDir, key + '.md'), 'w', encoding='utf8',
                     newline='\n') as file:
            file.write(page)
        written += 1
        indexText += key + '\n'

    with io.open(os.path.join(markdownDir, 'structuresIndex.md'), 'w', encoding='utf8',
                 newline='\n') as file:
        file.write(indexText + '```\n')

    print('structureDocsEmitter: ' + str(written + 1) + ' Markdown file(s) written')
    return 0


if __name__ == '__main__':
    sys.exit(main())
