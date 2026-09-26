#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  Emits the item reference manual from definitions/: docs/generated/items/*.md, one page
#           per item plus an index per item type (Markdown, which
#           replaced docs/theDoc/itemDefinition.tex and docs/RST/items/), and
#           docs/RST/confHelperItems.py (revision2026 step R4.3, part 2b). This is what remained of src/pythonGenerator/
#           pythonAutoGenerateObjects.py once its C++ headers, itemInterface.py and mini examples
#           had their own emitters; the code is unchanged apart from the moves. It reads the old
#           string records (definitionLoader), which is acceptable here: revision2026 step R7.1 replaces the
#           LaTeX/RST documentation pipeline as a whole.
#
# Usage:    python tools/generators/itemDocsEmitter.py
#
# Author:   Johannes Gerstmayr
# Date:     2018-05-18 (created as pythonAutoGenerateObjects.py), 2026-09-14 (emitter)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import os
import sys

toolsDirectory = os.path.dirname(os.path.abspath(__file__))
if toolsDirectory not in sys.path:
    sys.path.insert(0, toolsDirectory)

from autoGenerateHelper import ExtractExamplesWithKeyword, RemoveSpacesTabs, CountLines, \
    GenerateHeader, Str2Doxygen, GetDateStr, GetTypesStringLatex, \
    PyLatexRST, FileNameLower, RemoveIndentation


from autoGenerateHelper import KeywordExamplesMarkdown, MarkdownLabel, MarkdownHeading
from latexToMarkdown import NormalizeHeadings, DropRepeatedTitle, ConvertText as LatexText2Markdown
from userFunctionModel import ReadUserFunction

import copy
import os
import io #RST files written as UTF-8
import generatorPaths as paths
from exudynVersion import exudynVersionString
import definitionLoader

ADD_DOCSTRINGS = True

space4 = '    '
space8 = space4+space4
space12 = space8+space4

localListItemNames = [] #string list for highlighting

# compute destination number of str given from [C|M][P]
# [sParamComp=0, sParamMain=1, sComp=2, sMain=3]
# return -1 if no destination

#the item type tables and predicates live in tools/generators/itemModel.py (revision2026 step R4.3, part 2b)
from itemModel import possibleTypes, useNewUserFunctions, pyFunctionTypeConversion, pyFunctionTypeConversionUFtemplate, \
    IsASafelyVector, IsAVector, \
    IsASimpleMatrix, IsAMatrixVectorSpecial, IsAArrayIndex, IsASetSafelyParameter, \
    GetSetSafelyFunctionName, IsInternalSetGetParameter, IsTypeWithRangeCheck, \
    IsItemIndex, ExtractLatexSymbol


def RemoveIndentation2(text, addSpaces = '', removeAllSpaces = True, removeIndentation = True):
    lines=text.replace('\t','    ').split('\n')
    s = ''
    hasEndl = False
    if lines[-1] == '':
        hasEndl = True
        del lines[-1]
    
    if not removeAllSpaces:
        #a line that is nothing but spaces has no indentation: counting it made the minimum the
        #length of the shortest stray blank line - one space in several item descriptions - so the
        #text was dedented by one and everything that depends on the indentation was off (#2655)
        minIndent = 10000
        for line in lines:
            if line.strip() != '':
                nSpaces = len(line)-len(line.lstrip(' '))
                minIndent=min(minIndent, nSpaces)
        if minIndent == 10000:
            minIndent = 0

        if removeIndentation:
            for i, line in enumerate(lines):
                lines[i] = line[minIndent:]
    else:
        for i, line in enumerate(lines):
            lines[i] = line.lstrip()
        
    for i, line in enumerate(lines):
        s += addSpaces+line
        if i < len(lines)-1:
            s += '\n'

    if hasEndl: 
        s+='\n' #in this case, we had an endline and like to keep it

    return s #omit last \n



fileWriteCnt = 0
parameterCnt = 0 #counting total parameters
#************************************************
def UserFunctionDocumentation(parameter):
    """the description block of one user function, as a definition file would have written it

    The block is generated from the Python def the parameter carries: the signature line from its
    arguments, the prose from its docstring, the table from the annotations and the Args:/Returns:
    lines, and the example from userFunctionExample (revision2026b step RG12.4, #2664). The text
    returned is Markdown in the form definitions/README.md describes, so it goes through the same
    converter as a hand-written description and the same constructs work in it."""
    userFunction = ReadUserFunction(parameter['userFunction'], parameter['pythonName'])

    text = '**Userfunction**: `' + userFunction.Signature() + '`\n'
    text += userFunction.summary + '\n'
    if userFunction.details != '':
        text += userFunction.details + '\n'
    #one space after the slash: 21 of the hand-written headers have two and three have one, and a
    #generated header is the same everywhere (revision2026b step RG12.4.5, #2664)
    text += '\n| arguments / return | type or size | description |\n|---|---|---|\n'
    for (name, annotation) in userFunction.arguments:
        text += ('| `' + name + '` | '
                 + userFunction.TypeAndSize(annotation, userFunction.argumentSize.get(name, ''))
                 + ' | ' + userFunction.argumentText.get(name, '') + ' |\n')
    if userFunction.returnType != '':
        text += ('| **return value** | '
                 + userFunction.TypeAndSize(userFunction.returnType, userFunction.returnSize)
                 + ' | ' + userFunction.returnText + ' |\n')
    if parameter['userFunctionExample'] != '':
        text += '\n*Example*:\n\n```python\n' + parameter['userFunctionExample'].strip('\n') + '\n```\n'
    return text


#create autogenerated .h  files for list of parameters
#creates computational and main class files including parameter classes
def WriteFile(parseInfo, parameterList):
    """the documentation parts of one item; its C++ headers are emitted by
    tools/generators/itemHeaderEmitter.py"""
    global fileWriteCnt
    global parameterCnt

    print('\rProcess file '+str(fileWriteCnt).zfill(3)+': class='+parseInfo['class']+' '*20, end='', flush=True)
    fileWriteCnt+=1

    classStr = parseInfo['class']

    classTypeStr = parseInfo['classType']
    sTypeName = classStr.replace(classTypeStr,'')

    #print('class type=',classTypeStr, ', class=', classStr)
    #************************************
    #Latex doc:
    plr = PyLatexRST()


    hasPybindInterface = False
    for parameter in parameterList:
        if (parameter['lineType'] == 'V') & (parameter['cFlags'].find('I') != -1): #only if it is a member variable
            hasPybindInterface = True
            parameterCnt += 1 #count all parameters


    if hasPybindInterface: #otherwise do not include the description into latex doc
        localListItemNames.append(parseInfo['class'])
        localListItemNames.append('V'+parseInfo['class'])
        if len(parseInfo['pythonShortName']) != 0:
            localListItemNames.append(parseInfo['pythonShortName'])
            localListItemNames.append('V'+parseInfo['pythonShortName'])
            
        
        descriptionStr = parseInfo['classDescription']

        plr.AddDocu(text=descriptionStr,
                    section=parseInfo['class'],
                    sectionLevel=1,
                    sectionLabel='sec:item:' + parseInfo['class'])


        cPLR = PyLatexRST()
        vPLR = PyLatexRST()

        cPLR.AddDocu('The item \\mybold{' + parseInfo['class'] + "} with type = '"+
                     sTypeName + "' has the following parameters:")
        vPLR.AddDocu('The item V' + parseInfo['class'] + ' has the following parameters:')

        cPLR.DefItemStartTable(classStr=parseInfo['class'])        
        vPLR.DefItemStartTable(classStr=parseInfo['class'])        
        
        # cLatex  = '\\vspace{12pt} \\noindent The item {\\bf ' + parseInfo['class'] + "} with type = '"
        # cLatex += sTypeName + "' has the following parameters:\\vspace{-1cm}\\\\ \n"
        
        # vLatex  = 'The item V' + parseInfo['class'] + ' has the following parameters:\\vspace{-1cm}\\\\ \n'
        
        # sTemp   = '%reference manual TABLE\n'
        # sTemp  += '\\begin{center}\n'
        # sTemp  += '  \\footnotesize\n'
        # sTemp  += '  \\begin{longtable}{| p{4.5cm} | p{2.5cm} | p{0.5cm} | p{2.5cm} | p{6cm} |}\n'
        # sTemp  += space4+'\\hline\n'
        # sTemp  += space4+'\\bf Name & \\bf type & \\bf size & \\bf default value & \\bf description \\\\ \\hline\n'
        
        # cLatex += sTemp
        # vLatex += sTemp
    
        requestedMarkerString = ''
        itemTypeString = '' #string containing type of item (out of possibleTypes dict)
        requestedNodeString = ''


        for parameter in parameterList:
            if (parameter['lineType'].find('V') != -1) & (parameter['cFlags'].find('I') != -1): #also include parent class members!
                sString = ''
                if (parameter['type'] == 'String'):
                    sString="'"
                #write latex doc:
                parameterDescription = parameter['parameterDescription']
                [parameterDescription, latexSymbol] = ExtractLatexSymbol(parameterDescription)
                if parameter['cFlags'].find('Q') != -1: #CFMustBeGiven: the default is only a placeholder
                    parameterDescription += '; \mybold{must be given}: the default is only a placeholder'
                if latexSymbol.count('\\n'):
                    print('WARNING: found \\n in latexSymbol: '
                          +parseInfo['class']+':'+parameter['pythonName'])
                
                parameterTypeStr = parameter['type']
                parameterSizeStr = parameter['size']
                #the C++ literal decides the layout, as it always has; what is SHOWN is the
                #document rendering the definition carries (revision2026b step RG3.24.3, #2682)
                parameterDefaultValueStr = parameter['defaultValueDocument']
                if len(parameterTypeStr) > 35 or len(parameter['defaultValue']) > 17:
                    parameterDescription = '\\tabnewline ' + parameterDescription 

                if len(parameterTypeStr) > 15:
                    parameterSizeStr = '\\tabnewline ' + parameterSizeStr 
                if len(parameterTypeStr) > 18:
                    parameterDefaultValueStr = '\\tabnewline ' + parameterDefaultValueStr 

                if parameter['destination'].find('V') != -1: #visualization
                    thisPLR = vPLR
                else:
                    thisPLR = cPLR

                thisPLR.ItemInterfaceWriteRow(pythonName = parameter['pythonName'], 
                                              typeName = parameterTypeStr, 
                                              sSize = parameterSizeStr,
                                              sDefaultVal = sString+parameterDefaultValueStr+sString, 
                                              sSymbol = latexSymbol.replace('\n','\\n'), #correct e.g. \nu
                                              description = parameterDescription)

            elif (parameter['pythonName'] == 'GetRequestedMarkerType'):
                requestedMarkerString = GetTypesStringLatex(parameter['defaultValue'],'Marker', possibleTypes['Marker'],' +')
            elif (parameter['pythonName'] == 'GetRequestedNodeType'):
                requestedNodeString = GetTypesStringLatex(parameter['defaultValue'],'Node', possibleTypes['Node'],' +')
            elif (parameter['pythonName'] == 'GetType'):
                searchType = parseInfo['classType']
                if parseInfo['classType']=='Object': searchType += 'Type'
                itemTypeString = GetTypesStringLatex(parameter['defaultValue'],searchType, possibleTypes[parseInfo['classType']])
                #print(parseInfo['classType']+':'+itemTypeString)

        #cPLR.sLatex += space4+'visualization & V' + parseInfo['class'] + ' & & & parameters for visualization of item \\\\ \\hline\n'

        cPLR.ItemInterfaceWriteRow(pythonName = 'visualization', 
                                   typeName = 'V' + parseInfo['class'], sSize = '', sDefaultVal = '',
                                   description = 'parameters for visualization of item')

        cPLR.DefLatexFinishTable()
        vPLR.DefLatexFinishTable()

        #now assemble visualization and computation tables:

        if len(parseInfo['author']) != 0:
            pluralAuthors = ''
            if ',' in parseInfo['author']:
                pluralAuthors ='s'
            plr.AddDocu('Author'+pluralAuthors+': ' + parseInfo['author'] + '\n')

        if len(requestedMarkerString) + len(itemTypeString) + len(parseInfo['pythonShortName']) !=0:
            lstAdd = []
            plr.AddDocu('\\mybold{Additional information for ' + parseInfo['class'] + '}:\n', preNewLine=True)
            if len(itemTypeString) != 0:
                lstAdd += ['This \\texttt{' + parseInfo['classType'] + '} has/provides the following types = ' + itemTypeString]

            if len(requestedMarkerString) != 0:
                lstAdd += ['Requested \\texttt{Marker} type = ' + requestedMarkerString]
            if len(requestedNodeString) != 0:
                if requestedNodeString.find('_None') != -1:
                    lstAdd += ['Requested \\texttt{Node} type: read detailed information of item']
                else:
                    lstAdd += ['Requested \\texttt{Node} type = ' + requestedNodeString]
            if len(parseInfo['pythonShortName']) != 0:
                lstAdd += ['{\\bf Short name} for Python = \\texttt{' + parseInfo['pythonShortName'] + '}']
                lstAdd += ['{\\bf Short name} for Python visualization object = \\texttt{V' + parseInfo['pythonShortName'] + '}']

            plr.AddDocuList(lstAdd)

        plr += cPLR
        plr += vPLR

#        if len(parseInfo['outputVariables']) != 0:
#            dictOV = eval(parseInfo['outputVariables']) #output variables are given as a string, representing a dictionary with OutputVariables and descriptions
#            for outputVariables in dictOV.items(): 
#            

        #++++++++++++++++++++++++++++++++++++++++++++++
        #input parameters: only in latex table
        #addLatex = '' 
        plrAdd = PyLatexRST() #only added if non-empty

        #++++++++++++++++++++++++++++++++++++++++++++++
        #process outputVariables, including symbols
        if len(parseInfo['outputVariables']) != 0:
            plrAdd.AddDocu('\\mybold{The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions}:')
            plrAdd.DefLatexStartTable3(['output variable','symbol','description'])        

            #print("dict=",parseInfo['outputVariables'].replace('\\','\\\\'))
            dictOV = eval(parseInfo['outputVariables'].replace('\n','\\n').replace('\\','\\\\')) #output variables are given as a string, representing a dictionary with OutputVariables and descriptions
            for outputVariables in dictOV.items(): 
                #the name of an output variable is a NAME: Coordinates_t, not Coordinates\_t; the
                #escape was LaTeX and a Markdown page shows it as the underscore it stands for,
                #which is why it went unnoticed (revision2026b step RG3.14.14, #2677)
                oVariable = outputVariables[0]
                description = outputVariables[1]
                [description, latexSymbol] = ExtractLatexSymbol(description)
                if len(latexSymbol) != 0: 
                    latexSymbol = latexSymbol
                plrAdd.Table3WriteRow(cols=[oVariable, latexSymbol, description])
            
            plrAdd.DefLatexFinishTable()

        #++++++++++++++++++++++++++++++++++++++++++++++
        #the equations; everything before the %%RSTCOMPATIBLE marker is what the web
        #documentation shows, and the marker is the author's own judgement of where the LaTeX
        #stops carrying over
        #the equations. A %%RSTCOMPATIBLE marker used to say where the published part ended,
        #and it decided more than it said: this emission sat inside "if the marker is present",
        #so an item without one published no description at all. The whole text is published
        #now and the markers are gone (revision2026b step RG3.14.7.5, #2655).
        if len(parseInfo['equations']) != 0:
            plrAdd.sMarkdown += LatexText2Markdown(
                RemoveIndentation2(parseInfo['equations'], removeAllSpaces=False)) + '\n\n'

        #the user functions of the item, in the order of the parameters; a parameter that carries a
        #Python def has its block generated instead of written (revision2026b step RG12.4, #2664)
        for parameter in parameterList:
            if 'userFunction' in parameter:
                plrAdd.sMarkdown += LatexText2Markdown(
                    UserFunctionDocumentation(parameter)) + '\n\n'

        if len(parseInfo['miniExample']) != 0:
            plrAdd.AddDocu('', section='MINI EXAMPLE for ' + parseInfo['class'], sectionLevel=3, 
                        sectionLabel='miniExample_'+parseInfo['class'], preNewLine = True)
            plrAdd.AddDocuCodeBlock(parseInfo['miniExample'])

        plrAdd.sMarkdown += KeywordExamplesMarkdown(parseInfo['classType'],
                                                    parseInfo['class'],
                                                    parseInfo['pythonShortName'])

        #the equations, the output variables, the mini example and the examples, under their own
        #DESCRIPTION heading
        if len(plrAdd.sMarkdown.strip()) != 0:
            plr.sMarkdown += '\n' + MarkdownLabel('description_'+parseInfo['class']) + '\n'
            plr.sMarkdown += MarkdownHeading('DESCRIPTION of ' + parseInfo['class'], 2) + '\n\n'
            plr.sMarkdown += plrAdd.sMarkdown

    return [classTypeStr, plr.sMarkdown]


#%%**********************************************
#MAIN CONVERSION
#************************************************
    

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the Markdown pages of the item reference manual, replacing
#docs/theDoc/itemDefinition.tex and docs/RST/items/
def MarkdownBanner(title, label=''):
    return ('<!-- GENERATED by tools/generators/itemDocsEmitter.py from definitions/ '
            '- do not edit -->\n'
            + (label + '\n' if label != '' else '')
            + '# ' + title + '\n\n')


def ItemTypeFileName(key):
    """the page of one item type: 'Objects (Body)' -> 'objectBodyIndex'"""
    return FileNameLower(key.replace('(', '').replace(')', '').replace(' ', '')) + 'Index'


def WriteMarkdownPages(markdownItemList, folderDict, typeConversion, itemIntros, latexIntro):
    markdownDir = os.path.join(paths.repositoryRoot, 'docs', 'generated', 'items')
    os.makedirs(markdownDir, exist_ok=True)

    def Write(fileName, text):
        with io.open(os.path.join(markdownDir, fileName), 'w', encoding='utf8',
                     newline='\n') as file:
            file.write(text)

    #the chapter index
    indexText = ('<!-- GENERATED by tools/generators/itemDocsEmitter.py from definitions/ '
                 '- do not edit -->\n'
                 + MarkdownLabel('sec:item:reference:manual') + '\n'
                 '# Items reference manual\n\n'
                 'Reference manual for: objects, nodes, markers, loads and sensors\n\n'
                 + LatexText2Markdown(latexIntro) + '\n\n'
                 '```{toctree}\n:maxdepth: 2\n\n')
    for key in folderDict:
        indexText += ItemTypeFileName(key) + '\n'
    Write('itemsIndex.md', indexText + '```\n')

    written = 1
    for key in folderDict:
        typeText = (MarkdownBanner(key)
                    + LatexText2Markdown(itemIntros[typeConversion[key]]) + '\n\n'
                    + '```{toctree}\n:maxdepth: 2\n\n')

        for (classType, className, text) in markdownItemList:
            if classType != key:
                continue
            #the figures of an item live with the LaTeX chapters until R7.1.7 moves them to
            #docs/figures/; a leading slash resolves against the documentation source directory
            text = text.replace('](docs/figures/', '](/docs/figures/')
            #the page opened with "# ObjectGround" and then "## ObjectGround"; the second one
            #carried sec:item:<Item>, which every item reference points at, so the label moves to
            #the title and the heading goes (revision2026b step RG3.18, #2660)
            (label, body) = DropRepeatedTitle(text.strip(), className)
            Write(className + '.md',
                  NormalizeHeadings(MarkdownBanner(className, label) + body) + '\n')
            written += 1
            typeText += className + '\n'

        Write(ItemTypeFileName(key) + '.md', typeText + '```\n')
        written += 1

    print('itemDocsEmitter: ' + str(written) + ' Markdown file(s) written')


def main():
    #create Python/pybind11 file; currently not used ...
    #    pybindFile = 'pybind_objects.h'
    #    file=open(pybindFile,'w')  #clear file by one write access
    #    file.write('// # ++++++++++++++++++++++\n')
    #    file.write('// # pybind11 OBJECT includes; generated by Johannes Gerstmayr\n')
    #    file.write('// # ++++++++++++++++++++++\n')
    #    file.close()

    print('*************************')
    print('Autogenerate object files')

    directoryString = paths.autogeneratedDir
    
    #read system definition
    totalNumberOfLines = 0        #count number of lines generated automatically ...
    totalNumberOfFilesChanged = 0 #count how many files have been changed


    #the following commands are recognized:
    parseInfo = {'class':'',            # C++ class name
                 'writeFile':'',        #True; initiates finalization of class definition and file writing
                 'excludeFromTheDoc':'',#if True, this class will not generate latex docu (e.g. for experimental classes)
                 #'writePybindIncludes':'',#True, if pybind11 includes shall be written for this class
                 'cParentClass':'',     #name of parent computational object class or empty
                 'cBaseClass':'',       #name of computational object base class or empty
                 'mainParentClass':'',  #name of parent MainObject class or empty
                 'visuParentClass':'',  #name of parent VisualizationItem class or empty
                 'pythonShortName':'',  #short name for python interface
                 'addProtectedC':'',    #code added at protected section (e.g. constants)
                 'addPublicC':'',       #code added at protected section (e.g. constants or functions)
                 'addProtectedMain':'', #code added at protected section (e.g. constants)
                 'addPublicMain':'',    #code added at protected section (e.g. constants or functions)
                 'addIncludesC':'',     #code added at includes section (e.g. special base class)
                 'author':'',           #mentioned in C++ and in .tex files
                 'addIncludesMain':'',     #code added at includes section (e.g. special base class)
                 'classType':'',        #type of class: Object, Node, Sensor, Marker, Load, Sensor
                 'objectType':'',       #type of object, see sLatexObjectClass
                 'outputVariables':'',  #definition of output variables and description given as dictionary "{'OutputVariableType':'description ...', ...}"
                 'miniExample':'',      #mini python example (without headers and typical setup); code in separate lines, ended with '/end' in separate line
                 'equations':'',        #latex style equations, direct latex code; latex code in separate lines, ended with '/end' in separate line
                 'classDescription':''} #add a (brief, one line) description of class
    #this defines the columns of the line, which is then filled into this structure
    lineDefinition = ['lineType',       #[V|F[v]]P: V...Value (=member variable), F...Function (access via member function); v ... virtual Function; P ... write Pybind11 interface
                      'destination',    #M ... Main object, C ... computational object, V ... visualization object; P ... parameter structure
                      'pythonName',     #name which is used in python
                      'cplusplusName',     #name which is used in Exudyn (leave empty if it is the same)
                      'size',           #for size check; leave empty if size is non-constant; e.g. 3 (size of vector), 2x3 (2 rows, 3 columns)  %used for variables and vectors and matrices only!
                      'type',           #variable or return type: Bool, Int, Real, UInt, UReal, Vector, Matrix, SymmetricMatrix
                      'defaultValue',   #default value for member variable or function definition
                      'args',           # arguments in function declaration (empty for variable)
                      'cFlags',         # various flags: R(read only), M(modifiableDuringSimulation), N(parameter change needs object reset), C(onst member function),  D(declaration only; implementation in .cpp file done manually), O ... optional parameter in dictionary (otherwise using default value)
                                        #     P ... write Pybind11 interface, [default is read/write access and that changes are immediately applied and need no reset of the system]
                      'parameterDescription'] #description for parameter used in C++ code
    nparam = len(lineDefinition)
    
    
    mode = 0 #1...read parameterlist , 0...read definitions
    linecnt = 1
    
    parameterList = [] #list of dictionaries for parameters
    continueOperation = True #flag to signal that operation shall be terminated

    #++++++++++++++++++++++++++    
    sLatexObjectClass = ['Body','SuperElement','FiniteElement','Joint','Connector','Constraint','Object']
    sPythonGlobalNames = ['Node','Object','Marker','Load','Sensor']  #global python interface class types
    nObjectTypes = len(sLatexObjectClass)
    nPythonGlobal = len(sPythonGlobalNames)
    nLatexGlobal = nPythonGlobal+nObjectTypes
    sLatexGlobal = ['']*nLatexGlobal        #gobal Latex string; 'Node','Object','Marker','Load','Sensor'
    
    #... convert to dictionary, in or to be safe w.r.t. relation to sLatexGlobalNames
    sLatexGlobalItemIntros={'Nodes':                   'Nodes provide coordinates for objects. Loads can be applied and Markers or Sensors can be attached to Nodes. The sorting of Nodes in the system (the order they are added to mbs) defines the order of system coordinates.',
                            'Objects (Body)':          'A Body is a special Object, which has physical properties such as mass. A localPosition can be measured w.r.t.\\ the reference point of the body',
                            'Objects (SuperElement)':  'A SuperElement is a special Object which acts on a set of nodes. Essentially, SuperElements can be linked with special SuperElement markers. SuperElements may represent complex flexible bodies, based on finite element formulations.',
                            'Objects (FiniteElement)': 'A FiniteElement is a special Object and Body, which is used to define deformable bodies, such as beams or solid finite elements. FiniteElements are usually linked to two or more nodes.',
                            'Objects (Joint)':         'A Joint is a special Object, Connector and Constraint, which is attached to position or rigid body markers. The joint results in special algebraic equations and requires implicit time integration. Joints represent special constraints, as described in multibody system dynamics literature.',
                            'Objects (Connector)':     'A Connector is a special Object, which links two or more markers. A Connector which is not a Constraint, is a force element (e.g., spring-damper) or a penalty based joint.',
                            'Objects (Constraint)':    'A Constraint is a special Object and Connector, which links two or more markers. A Constraint leads to algebraic equations, which exactly fulfill special constraints on the kinematic behavior of the multibody syste, such as a constraint on a coordinate or a distance constraint.',
                            'Objects (Object)':        'A Object provides equations, using coordinates from Nodes. General objects lead to system equations, that do not represent physical Bodies or Connectors.',
                            'Markers':                 'A Marker provides an interface BETWEEN a large variety of Nodes / Bodies / Objects AND Connectors / Loads. To understand which markers are needed, see first the requested \\texttt{Marker} type of the connector, constraint or joint. Hereafter, chose a \\texttt{Marker} -- attached to a node, body or object -- with the according properties. The \\texttt{Marker} may provide more information (e.g., position and orientation) than needed.',
                            'Loads':                   'A Load applies a (usually constant) force, torque, mass-proportional or generalized load onto Nodes or Objects via Markers. The requested \\texttt{Marker} types need to be provided by the used \\text{Marker}. The marker may provide more types than requested. For non-constant loads, use either a \\texttt{load...UserFunction} or change the load in every step by means of a \\texttt{preStepUserFunction} in the \\texttt{MainSystem} (mbs).',
                            'Sensors':                 'A Sensor is used to measure quantities during simulation. Sensors may be attached to Nodes, Objects, Markers or Loads. Sensor values may be directly read via mbs or can be continuously written to files or SensorRecorder during simulation. The exudyn.plot Python utility function PlotSensor(...) can be conveniently used to show Sensor values over time.',
                            }

    latexGlobalFromPython = [0,1,nObjectTypes+1,nObjectTypes+2,nObjectTypes+3]
    sLatexGlobalNames = ['Nodes']
    objectClassDict = {} #convert objectType to objectClass number
    symbolicUserFunctionSet = [] #for both set and transfer of symbolic user functions 

    #manually add MainSystem user functions => see MainSystemUserFunctions:
    symbolicUserFunctionSet.append({'itemType': 'MainSystem',
                                    'classType': '', 
                                    'userFunctionName': 'preStepUserFunction', 
                                    'pyUserFunctionType': 'PyFunctionBoolMbsScalar'})
    symbolicUserFunctionSet.append({'itemType': 'MainSystem',
                                    'classType': '', 
                                    'userFunctionName': 'postStepUserFunction', 
                                    'pyUserFunctionType': 'PyFunctionBoolMbsScalar'})
    symbolicUserFunctionSet.append({'itemType': 'MainSystem',
                                    'classType': '', 
                                    'userFunctionName': 'postNewtonFunction', 
                                    'pyUserFunctionType': 'PyFunctionVector2DMbsScalar'})
    
    #create dictionaries for storing item information (in particular for auto-registration)
    globalItemsDict = {}
    for item in sPythonGlobalNames:
        globalItemsDict[item] = {}
    
    #other lists for documentation:
    for oi, oClass in enumerate(sLatexObjectClass):
        sLatexGlobalNames += ['Objects ('+oClass+')']
        objectClassDict[oClass] = oi

    sLatexGlobalNames += ['Markers','Loads','Sensors']

    #++++++++++++++++++++++++++    
    #Latex and RST
    sRSTItemList = []   #list of class type, class name, RST string
    sMarkdownItemList = [] #the same, in Markdown
    sRSTfolderDict = {} #dict containing available folders (to create index file)
    sRSTtypeConversion = {} #conversion from singular to plural
    
    #++++++++++++++++++++++++++    
    
    multiLineReading = False #for equations and miniExample
    multiLineString = '' #stored string from multiline reading
    multiLineType = ''   #equations or miniExample
    cnt = 0
    
    #the definitions come from definitions/: definitionLoader yields
    #each class in the form the old line parser built it, and the code below is what that
    #parser ran every time it reached writeFile
    parseInfoTemplate = copy.deepcopy(parseInfo)
    for parseInfo, parameterList in definitionLoader.LoadItemDefinitions(parseInfoTemplate, lineDefinition):
        #++++++++++++++++++++++++++++++
        #now write C++ header file for defined class
        #print(parseInfo)
        (classTypeStr, markdownText) = WriteFile(parseInfo, parameterList)

        #+++++++++++++++++++++++++++++++
        className = parseInfo['class']
        classType = parseInfo['classType']
        classNamePure = 'Invalid'
        for key, value in globalItemsDict.items():
            if className.startswith(key):
                classNamePure = className[len(classType):]

        globalItemsDict[classType][classNamePure] = {} #add new dictionary for class
        #+++++++++++++++++++++++++++++++

        #find index of python objects
        typeInd = -1
        it = 0
        for item in sPythonGlobalNames: 
            if item == classTypeStr:
                typeInd = it
            it+=1


        if typeInd == -1:
            print("ERROR: no valid base name found")
        else:

            if parseInfo['excludeFromTheDoc'] != 'True':
                sRSTtype = parseInfo['classType']
                sRSTtype2 = parseInfo['classType']+'s'
                if parseInfo['classType'] == 'Object':
                    sRSTtype += ' ('+parseInfo['objectType']+')'
                    sRSTtype2 += ' ('+parseInfo['objectType']+')'

                sMarkdownItemList += [(sRSTtype, parseInfo['class'], markdownText)]
                if sRSTtype not in sRSTfolderDict:
                    sRSTfolderDict[sRSTtype] = []
                    sRSTtypeConversion[sRSTtype] = sRSTtype2 #conversion from singular to plural
                sRSTfolderDict[sRSTtype] += [parseInfo['class']]

            # print('item=',parseInfo['class'], ', typeInd=',typeInd,',objType=', oType, ', indexGlobal=', indexLatexGlobal)


        #++++++++++++++++++++++++++++++
        #++++++++++++++++++++++++++++++
    
    print('') #endline after counting ...

    if (continueOperation == False):
        print('\n\nERROR: Parsing terminated unexpectedly in line',linecnt,'\n\n')
        
    print("parsed a total of", linecnt, "lines")


#    sLatexItemList = '\n\\mysubsection{List of Items}\nThe following items are available in \codeName:\n\\begin{itemize}\n' + sLatexItemList
#    sLatexItemList += '\\end{itemize}\n'
    #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    print('total number of lines generated =',totalNumberOfLines)
    print('total number of files changed =',totalNumberOfFilesChanged)

    #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #docs/theDoc/itemDefinition.tex is not written; the LaTeX
    #string is still built, and dies with the LaTeX branch in R7.1.7

    sLatexIntro=r"""
This chapter includes the reference manual for all objects (bodies/constraints), nodes, markers, loads and sensors (\mybold{= items}).
For description of types (e.g., the meaning of \texttt{Vector3D} or \texttt{NumpyMatrix}), see \refSection{sec:typesDescriptions}.

"""


    #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #write the Markdown pages: one per item, one index per item type
    #and the chapter index, which carries the chapter label. docs/RST/items/ is gone with this
    #step; the RST strings are still built and die with the RST branch in R7.1.7.
    WriteMarkdownPages(sMarkdownItemList, sRSTfolderDict, sRSTtypeConversion,
                       sLatexGlobalItemIntros, sLatexIntro)

    #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #write class names for confHelperItems.py
    
    sConfHelper = ''
    sConfHelper += '#this is a helper file to define additional keywords for examples\n'
    sConfHelper += '#Created: 2023-03-17, Johannes Gerstmayr\n\n'
    
    #list of classes and enum classes:
    sConfHelper += 'listItemNames=['
    for s in localListItemNames:
        sConfHelper += "'" + s + "'" + ', '
    sConfHelper += ']\n\n'

    with open(paths.generatedDir+'confHelperItems.py', 'w',encoding='utf8') as f:
        f.write(sConfHelper)

    print('total parameters converted:', parameterCnt)
    return 0


if __name__ == '__main__':
    sys.exit(main())


