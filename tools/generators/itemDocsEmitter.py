#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  Emits the item reference manual from definitions/: docs/generated/items/*.md, one page
#           per item plus an index per item type (Markdown, which
#           replaced docs/theDoc/itemDefinition.tex and docs/RST/items/), and
#           docs/RST/confHelperItems.py. This is what remained of src/pythonGenerator/
#           pythonAutoGenerateObjects.py once its C++ headers, itemInterface.py and mini examples
#           had their own emitters; the code is unchanged apart from the moves. It reads the
#           string records of definitionLoader.
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
    GenerateHeader, Str2Doxygen, GetDateStr, GetTypesStringDocu, \
    DeclarationWriter, FileNameLower, RemoveIndentation


from autoGenerateHelper import KeywordExamplesMarkdown, MarkdownLabel, MarkdownHeading
from latexToMarkdown import NormalizeHeadings, DropRepeatedTitle, ConvertText as LatexText2Markdown
from userFunctionModel import ReadUserFunction

import copy
import re
import os
import io #RST files written as UTF-8
import generatorPaths as paths
from exudynVersion import exudynVersionString
import definitionLoader
import itemCompatibility

#which items fit together, from their declared types, for the Interface block of each page (#2725)
compatibilityItems = itemCompatibility.LoadItems()
compatibilityByName = dict((item.name, item) for item in compatibilityItems)

ADD_DOCSTRINGS = True

space4 = '    '
space8 = space4+space4
space12 = space8+space4

localListItemNames = [] #string list for highlighting

# compute destination number of str given from [C|M][P]
# [sParamComp=0, sParamMain=1, sComp=2, sMain=3]
# return -1 if no destination

#the item type tables and predicates live in tools/generators/itemModel.py
from itemModel import possibleTypes, useNewUserFunctions, pyFunctionTypeConversion, pyFunctionTypeConversionUFtemplate, \
    IsASafelyVector, IsAVector, \
    IsASimpleMatrix, IsAMatrixVectorSpecial, IsAArrayIndex, IsASetSafelyParameter, \
    GetSetSafelyFunctionName, IsInternalSetGetParameter, IsTypeWithRangeCheck, \
    IsItemIndex, ExtractMathSymbol


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
    lines, and the example from userFunctionExample (#2664). The text
    returned is Markdown in the form definitions/README.md describes, so it goes through the same
    converter as a hand-written description and the same constructs work in it."""
    userFunction = ReadUserFunction(parameter['userFunction'], parameter['pythonName'])

    text = '**Userfunction**: `' + userFunction.Signature() + '`\n'
    text += userFunction.summary + '\n'
    if userFunction.details != '':
        text += userFunction.details + '\n'
    #one space after the slash: 21 of the hand-written headers have two and three have one, and a
    #generated header is the same everywhere (#2664)
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
#the definitions as written, for the fields that are lists
rawItemDefinitions = dict((definition['className'], definition) for moduleName in definitionLoader.itemModules
                          for definition in __import__(moduleName).definitions)
mainSystemExtensionsText = io.open(os.path.join(paths.pythonDir, 'exudyn', 'misc', 'mainSystemExtensions.py'),
                                   encoding='utf-8').read()


def CreateFunctionsMarkdown(definition):
    """the hint that a Create function adds this item with what it needs - read before the parameters (#2737)"""
    names = definition.get('createFunctions') or []
    for name in names:
        if 'def MainSystem' + name + '(' not in mainSystemExtensionsText:
            raise ValueError(definition['className'] + ': createFunctions names ' + name
                             + ', which is no function of exudyn.misc.mainSystemExtensions')
    if len(names) == 0:
        return ''
    links = ['[`mbs.' + name + '`](#sec-mainsystemextensions-' + name.lower() + ')' for name in names]
    return ('\n**Simpler**: ' + ' or '.join(links) + ' add' + ('s' if len(names) == 1 else '')
            + ' this item, with what it needs, in one call.\n\n')


def ItemExamples(definition):
    """the examples a definition names, as paths relative to python/; None if it names none (#2737)"""
    examples = definition.get('examples')
    if examples is None:
        return None
    for name in examples:
        if not os.path.isfile(os.path.join(paths.pythonDir, name)):
            raise ValueError(definition['className'] + ': the example ' + name + ' does not exist in python/')
    return examples


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
    writer = DeclarationWriter()


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
            
        
        descriptionStr = parseInfo['overallDescription']

        writer.AddDocu(text=descriptionStr,
                    section=parseInfo['class'],
                    sectionLevel=1,
                    sectionLabel='sec:item:' + parseInfo['class'])


        cWriter = DeclarationWriter()
        vWriter = DeclarationWriter()

        cWriter.AddDocu('The parameters of the item; in a dictionary, its type is ' + "'" + sTypeName + "':",
                        section='Parameters', sectionLevel=1)
        vWriter.AddDocu('The parameters of `V' + parseInfo['class'] + '`, given as `visualization`:',
                        section='Visualization parameters', sectionLevel=1)

        cWriter.DefItemStartTable(classStr=parseInfo['class'])        
        vWriter.DefItemStartTable(classStr=parseInfo['class'])        
        
        requestedMarkerString = ''
        itemTypeString = '' #string containing type of item (out of possibleTypes dict)
        requestedNodeString = ''


        deprecatedNames = [] #renamed parameters: listed below the table, not in it (#2589)
        for parameter in parameterList:
            if parameter.get('deprecated') is not None and parameter['lineType'].find('V') != -1:
                deprecatedNames.append('`' + parameter['pythonName'] + '` (deprecated since ' + str(parameter['deprecated'].since)
                                       + ', removed in ' + str(parameter['deprecated'].expires) + '): use `'
                                       + parameter['parameterDescription'] + '`')
            elif (parameter['lineType'].find('V') != -1) & (parameter['cFlags'].find('I') != -1): #also include parent class members!
                sString = ''
                if (parameter['type'] == 'String'):
                    sString="'"
                #write latex doc:
                parameterDescription = parameter['parameterDescription']
                [parameterDescription, mathSymbol] = ExtractMathSymbol(parameterDescription)
                if parameter['cFlags'].find('Q') != -1: #CFMustBeGiven: the default is only a placeholder
                    parameterDescription += '; \mybold{must be given}: the default is only a placeholder'
                if mathSymbol.count('\\n'):
                    print('WARNING: found \\n in mathSymbol: '
                          +parseInfo['class']+':'+parameter['pythonName'])
                
                parameterTypeStr = parameter['type']
                parameterSizeStr = parameter['size']
                #the C++ literal decides the layout, as it always has; what is SHOWN is the
                #document rendering the definition carries (#2682)
                parameterDefaultValueStr = parameter['defaultValueDocument']
                if len(parameterTypeStr) > 35 or len(parameter['defaultValue']) > 17:
                    parameterDescription = '\\tabnewline ' + parameterDescription 

                if len(parameterTypeStr) > 15:
                    parameterSizeStr = '\\tabnewline ' + parameterSizeStr 
                if len(parameterTypeStr) > 18:
                    parameterDefaultValueStr = '\\tabnewline ' + parameterDefaultValueStr 

                if parameter['destination'].find('V') != -1: #visualization
                    thisWriter = vWriter
                else:
                    thisWriter = cWriter

                thisWriter.ItemInterfaceWriteRow(pythonName = parameter['pythonName'], 
                                              typeName = parameterTypeStr, 
                                              sSize = parameterSizeStr,
                                              sDefaultVal = sString+parameterDefaultValueStr+sString, 
                                              sSymbol = mathSymbol.replace('\n','\\n'), #correct e.g. \nu
                                              description = parameterDescription)

            elif (parameter['pythonName'] == 'GetRequestedMarkerType'):
                requestedMarkerString = GetTypesStringDocu(parameter['defaultValue'],'Marker', possibleTypes['Marker'],' +')
            elif (parameter['pythonName'] == 'GetRequestedNodeType'):
                requestedNodeString = GetTypesStringDocu(parameter['defaultValue'],'Node', possibleTypes['Node'],' +')
            elif (parameter['pythonName'] == 'GetType'):
                searchType = parseInfo['classType']
                if parseInfo['classType']=='Object': searchType += 'Type'
                itemTypeString = GetTypesStringDocu(parameter['defaultValue'],searchType, possibleTypes[parseInfo['classType']])
                #print(parseInfo['classType']+':'+itemTypeString)

        cWriter.ItemInterfaceWriteRow(pythonName = 'visualization', 
                                   typeName = 'V' + parseInfo['class'], sSize = '', sDefaultVal = '',
                                   description = 'parameters for visualization of item')

        cWriter.DefFinishTable()
        vWriter.DefFinishTable()
        if len(deprecatedNames) != 0:
            cWriter.sMarkdown += chr(10) + 'Renamed parameters, still taken with a `DeprecationWarning`: ' + '; '.join(deprecatedNames) + '.' + chr(10)

        #now assemble visualization and computation tables:

        if len(parseInfo['author']) != 0:
            pluralAuthors = ''
            if ',' in parseInfo['author']:
                pluralAuthors ='s'
            writer.AddDocu('Author'+pluralAuthors+': ' + parseInfo['author'] + '\n')

        #THE SIMPLER WAY FIRST: the Create functions that add this item, before the parameters (#2737)
        writer.sMarkdown += CreateFunctionsMarkdown(rawItemDefinitions[parseInfo['class']])

        #THE INTERFACE: the Python names and, in words, which items fit to this one - generated from
        #the types the definitions declare, instead of the type bits the reader had to match (#2725)
        interfaceLines = []
        if len(parseInfo['pythonShortName']) != 0:
            interfaceLines.append('Python names: `' + parseInfo['class'] + '` or `' + parseInfo['pythonShortName']
                                  + '`, and `V' + parseInfo['pythonShortName'] + '` for its visualization')
        interfaceLines += itemCompatibility.InterfaceLines(compatibilityByName[parseInfo['class']],
                                                           compatibilityItems)
        if requestedNodeString.find('_None') != -1:
            interfaceLines.append('Nodes: see the detailed description')
        if len(interfaceLines) != 0:
            writer.AddDocu('', section='Interface', sectionLevel=1)
            writer.AddDocuList(interfaceLines)

        writer += cWriter
        writer += vWriter

#        if len(parseInfo['outputVariables']) != 0:
#            dictOV = eval(parseInfo['outputVariables']) #output variables are given as a string, representing a dictionary with OutputVariables and descriptions
#            for outputVariables in dictOV.items(): 
#            

        #++++++++++++++++++++++++++++++++++++++++++++++
        #input parameters: only in latex table
        writerAdd = DeclarationWriter() #only added if non-empty

        #++++++++++++++++++++++++++++++++++++++++++++++
        #process outputVariables, including symbols
        writerOutput = DeclarationWriter()
        if len(parseInfo['outputVariables']) != 0:
            writerOutput.AddDocu('Available as `OutputVariableType` in sensors, `Get...Output()` and other functions:',
                                 section='Output variables', sectionLevel=1)
            writerOutput.DefStartTable3(['output variable','symbol','description'])        

            #print("dict=",parseInfo['outputVariables'].replace('\\','\\\\'))
            dictOV = eval(parseInfo['outputVariables'].replace('\n','\\n').replace('\\','\\\\')) #output variables are given as a string, representing a dictionary with OutputVariables and descriptions
            for outputVariables in dictOV.items(): 
                #the name of an output variable is a NAME: Coordinates_t, not Coordinates\_t; the
                #escape was LaTeX and a Markdown page shows it as the underscore it stands for,
                #which is why it went unnoticed (#2677)
                oVariable = outputVariables[0]
                description = outputVariables[1]
                [description, mathSymbol] = ExtractMathSymbol(description)
                writerOutput.Table3WriteRow(cols=[oVariable, mathSymbol, description])
            
            writerOutput.DefFinishTable()

        #++++++++++++++++++++++++++++++++++++++++++++++
        #the equations; everything before the %%RSTCOMPATIBLE marker is what the web
        #documentation shows, and the marker is the author's own judgement of where the LaTeX
        #stops carrying over
        #the equations. A %%RSTCOMPATIBLE marker used to say where the published part ended,
        #and it decided more than it said: this emission sat inside "if the marker is present",
        #so an item without one published no description at all. The whole text is published
        #now and the markers are gone (#2655).
        if len(parseInfo['detailedDescription']) != 0:
            writerAdd.sMarkdown += LatexText2Markdown(
                RemoveIndentation2(parseInfo['detailedDescription'], removeAllSpaces=False)) + '\n\n'

        #the user functions of the item, in the order of the parameters; a parameter that carries a
        #Python def has its block generated instead of written (#2664)
        for parameter in parameterList:
            if 'userFunction' in parameter:
                writerAdd.sMarkdown += LatexText2Markdown(
                    UserFunctionDocumentation(parameter)) + '\n\n'

        #the output variables, the detailed description with the user functions, the mini example
        #and the examples - each under a heading of its own (#2725)
        writer.sMarkdown += writerOutput.sMarkdown
        if len(writerAdd.sMarkdown.strip()) != 0:
            if not writer.sMarkdown.endswith('\n\n'):
                writer.sMarkdown += '\n'
            writer.sMarkdown += MarkdownLabel('description_'+parseInfo['class']) + '\n'
            writer.sMarkdown += MarkdownHeading('Detailed description', 1) + '\n\n'
            writer.sMarkdown += writerAdd.sMarkdown

        writerExamples = DeclarationWriter()
        if len(parseInfo['miniExample']) != 0:
            writerExamples.AddDocu('', section='Mini example', sectionLevel=1,
                                   sectionLabel='miniExample_'+parseInfo['class'], preNewLine = True)
            writerExamples.AddDocuCodeBlock(parseInfo['miniExample'])
        writerExamples.sMarkdown += KeywordExamplesMarkdown(parseInfo['classType'],
                                                         parseInfo['class'],
                                                         parseInfo['pythonShortName'],
                                                         ItemExamples(rawItemDefinitions[parseInfo['class']]))
        if len(writerExamples.sMarkdown.strip()) != 0:
            if not writer.sMarkdown.endswith('\n\n'):
                writer.sMarkdown += '\n'
            writer.sMarkdown += writerExamples.sMarkdown

    #one blank line before a heading or its target, however many the parts above end with
    writer.sMarkdown = re.sub(r'\n{3,}(?=\(|#{2,} )', '\n\n', writer.sMarkdown)
    return [classTypeStr, writer.sMarkdown]


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


#the title of the general page of a kind is "General info for all <plural>" (#2739)
kindPlural = {'Nodes': 'nodes', 'Objects (Body)': 'bodies', 'Objects (SuperElement)': 'super elements',
              'Objects (Object)': 'objects', 'Objects (FiniteElement)': 'finite elements',
              'Objects (Connector)': 'connectors', 'Objects (Constraint)': 'constraints',
              'Objects (Joint)': 'joints', 'Markers': 'markers', 'Loads': 'loads', 'Sensors': 'sensors'}


def WriteMarkdownPages(markdownItemList, folderDict, typeConversion, itemIntros, intro):
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
                 + LatexText2Markdown(intro) + '\n\n'
                 '```{toctree}\n:maxdepth: 2\n\n')
    for key in folderDict:
        indexText += ItemTypeFileName(key) + '\n'
    Write('itemsIndex.md', indexText + '```\n')

    written = 1
    for key in folderDict:
        kindEntry = itemIntros[typeConversion[key]]
        typeText = MarkdownBanner(key) + LatexText2Markdown(kindEntry['overallDescription']) + '\n\n'
        typeText += '```{toctree}\n:maxdepth: 1\n\n'
        if kindEntry['detailedDescription'].strip() != '':
            #the general section of the kind is a page of its own and the first entry of the kind,
            #so that it and every item are siblings in the navigation (#2739)
            generalName = ItemTypeFileName(key)[:-len('Index')] + 'General'
            generalText = (LatexText2Markdown(RemoveIndentation2(kindEntry['detailedDescription'],
                                                                 removeAllSpaces=False)) + '\n\n')
            if typeConversion[key] == 'Markers':
                #the table of all markers, generated from their declared types like the Interface
                #block of every page (#2725)
                generalText += (MarkdownHeading('All markers', 1) + '\n\n'
                                + itemCompatibility.MarkerTable(compatibilityItems) + '\n')
            (banner, rest) = MarkdownBanner('General info for all '
                                            + kindPlural[typeConversion[key]]).split('\n', 1)
            Write(generalName + '.md', banner + '\n```{raw} latex\n\\clearpage\n```\n\n' + rest
                  + generalText.rstrip('\n') + '\n')
            written += 1
            typeText += generalName + '\n'

        for (classType, className, text) in markdownItemList:
            if classType != key:
                continue
            #the figures of an item are in docs/figures/; a leading slash resolves against the
            #documentation source directory
            text = text.replace('](docs/figures/', '](/docs/figures/')
            #the page opened with "# ObjectGround" and then "## ObjectGround"; the second one
            #carried sec:item:<Item>, which every item reference points at, so the label moves to
            #the title and the heading goes (#2660)
            (label, body) = DropRepeatedTitle(text.strip(), className)
            page = NormalizeHeadings(MarkdownBanner(className, label) + body) + '\n'
            #EVERY ITEM ON A NEW PAGE of the PDF, which is how a reference manual is read (#2729);
            #the HTML build ignores a raw LaTeX block
            (banner, rest) = page.split('\n', 1)
            Write(className + '.md', banner + '\n```{raw} latex\n\\clearpage\n```\n\n' + rest)
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
                 'objectType':'',       #type of object, see objectClassNames
                 'outputVariables':'',  #definition of output variables and description given as dictionary "{'OutputVariableType':'description ...', ...}"
                 'miniExample':'',      #mini python example (without headers and typical setup); code in separate lines, ended with '/end' in separate line
                 'detailedDescription':'', #the full description of the page, after the generated part: Markdown with LaTeX mathematics (definitions/README.md)
                 'overallDescription':'', #the brief description: the class, the docstring, the paragraph under the heading
                 'requestedNodeTypes':'', #node markers: the node types they need (itemCompatibility.py)
                 'createFunctions':'',  #the mbs.Create... functions that add the item (#2737)
                 'examples':'',         #the examples of the page, instead of those found by name (#2737)
                 'miniExamplePerformanceTest':''} #the steps of the MiniExample's performance run (#2745); not on the page
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
    objectClassNames = ['Body','SuperElement','FiniteElement','Joint','Connector','Constraint','Object']
    sPythonGlobalNames = ['Node','Object','Marker','Load','Sensor']  #global python interface class types
    nObjectTypes = len(objectClassNames)
    
    #what all items of a kind have in common - the page of the kind - is a definition like the items
    #are, in definitions/itemKindDefinitions.py (#2725)
    globalItemIntros = dict((entry['kind'], entry) for entry in __import__('itemKindDefinitions').definitions)

    globalPageNames = ['Nodes']
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
    for oi, oClass in enumerate(objectClassNames):
        globalPageNames += ['Objects ('+oClass+')']
        objectClassDict[oClass] = oi

    globalPageNames += ['Markers','Loads','Sensors']

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

        #++++++++++++++++++++++++++++++
        #++++++++++++++++++++++++++++++
    
    print('') #endline after counting ...

    if (continueOperation == False):
        print('\n\nERROR: Parsing terminated unexpectedly in line',linecnt,'\n\n')
        
    print("parsed a total of", linecnt, "lines")


    #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    print('total number of lines generated =',totalNumberOfLines)
    print('total number of files changed =',totalNumberOfFilesChanged)

    #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #the introduction of the items chapter, converted by LatexText2Markdown

    itemsIntro=r"""
This chapter includes the reference manual for all objects (bodies/constraints), nodes, markers, loads and sensors (\mybold{= items}).
For description of types (e.g., the meaning of \texttt{Vector3D} or \texttt{NumpyMatrix}), see \refSection{sec:typesDescriptions}.

"""


    #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #write the Markdown pages: one per item, one index per item type
    #and the chapter index, which carries the chapter label.
    WriteMarkdownPages(sMarkdownItemList, sRSTfolderDict, sRSTtypeConversion,
                       globalItemIntros, itemsIntro)

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


