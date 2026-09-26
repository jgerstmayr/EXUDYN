#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  The reader of the docstrings (and @docmeta/@extends decorators) of the utility modules
#           of python/exudyn/, and the text helpers shared by mainSystemExtensionDocsEmitter.py
#           and utilityDocsEmitter.py. Moved out of src/pythonGenerator/utilitiesDocuGenerator.py
#           (revision2026 step R4.3, part 2e); reads docstrings instead of #** comments
#
# Usage:    import utilityDocsModel
#
# Author:   Johannes Gerstmayr
# Date:     2020-06-09 (created as utilitiesDocuGenerator.py), 2026-09-14 (utilityDocsModel.py)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import ast
import io
import re
import os
import sys

toolsDirectory = os.path.dirname(os.path.abspath(__file__))
if toolsDirectory not in sys.path:
    sys.path.insert(0, toolsDirectory)

import copy #for deep copies

import generatorPaths as paths                                                  # noqa: E402
from autoGenerateHelper import RemoveIndentation, Latex2RSTlabel, \
          DocStringGoogleFromPlainText                                          # noqa: E402

ADD_DOCSTRINGS = True

maxWarningsMutableArgs = 200 #warnings in case of list or dict default args (mutable args)
from autoGenerateHelper import MarkdownLabel, MarkdownHeading, LatexText2Markdown, \
                               KeywordExamplesMarkdown                             # noqa: E402
#list of functions for which mutable args have been checked:
mutableArgsFunctionsChecked = [
    'GenerateStraightLineANCFCable','GenerateStraightLineANCFCable2D','PointsAndSlopes2ANCFCable2D','GenerateCircularArcANCFCable2D', 'GenerateStraightBeam', #beams
    #FEM:
    'CreateReevingCurve', 'AddObjectFFRF','CMSObjectComputeNorm', 'AddObjectFFRFreducedOrderWithUserFunctions',
    'AddObjectFFRFreducedOrder', 'AddElasticSupportAtNode', 'CreateLinearFEMObjectGenericODE2', 
    'CreateNonlinearFEMObjectGenericODE2NGsolve', 'ComputeHurtyCraigBamptonModes',  
    #graphics:
    'Sphere','Line','Lines','Circle','Text','Cuboid','BrickXYZ','Brick','Cylinder','RigidLink','SolidOfRevolution',
    'Arrow','Basis','Frame','Quad','CheckerBoard','SolidExtrusion','FromPointsAndTrigs','FromSTLfileASCII',
    'FromSTLfile','Brick','Torus','Tube',
    #graphicsDataUtilities:
    'CirclePointsAndSegments', 'GraphicsDataRectangle', 'GraphicsDataOrthoCubeLines', 'AddEdgesAndSmoothenNormals',
    'BallBearingRings','InvoluteGear','ToothedRack',
    # 'GraphicsDataFromPointsAndTrigs', 
    # 'GraphicsDataLine', 'GraphicsDataCircle', 'GraphicsDataText', 'GraphicsDataRectangle', #4 x problems fixed using list()
    # 'GraphicsDataOrthoCubeLines', 'GraphicsDataOrthoCube', 'GraphicsDataOrthoCubePoint', 'GraphicsDataCube', #4 x problems fixed using list()
    # 'GraphicsDataSphere', 'GraphicsDataCylinder', 'GraphicsDataRigidLink', 'GraphicsDataFromSTLfileTxt', #4 x problems fixed using list()
    # 'GraphicsDataCheckerBoard', 'GraphicsDataArrow', 'GraphicsDataBasis', 'GraphicsDataFrame', #ok
    # 'GraphicsDataFromSTLfile', 'GraphicsDataSolidExtrusion', 'AddEdgesAndSmoothenNormals', #4 x problems fixed using list()
    # 'GraphicsDataSolidOfRevolution', 'GraphicsDataQuad', #2 x problems fixed using list()
    #interactive:
    'SolutionViewer',#'__init__',
    #kinematicTree:
    'ForwardDynamicsCRB', 'ComputeMassMatrixAndForceTerms', 'AddExternalForces',
    #mainSystemExtensions:
    'MainSystemCreateGround','MainSystemCreateMassPoint','MainSystemCreateRigidBody','MainSystemCreateSpringDamper','MainSystemCreateCartesianSpringDamper',
    'MainSystemCreateRigidBodySpringDamper', 'MainSystemCreateTorsionalSpringDamper', 'MainSystemCreateRevoluteJoint', 'MainSystemCreatePrismaticJoint',
    'MainSystemCreateSphericalJoint', 'MainSystemCreateGenericJoint', 'MainSystemCreateDistanceConstraint',
    'MainSystemCreateDistanceConstraint', 'MainSystemCreateRollingDiscPenalty', 'MainSystemCreateRollingDisc', 'MainSystemCreateKinematicTree',
    'MainSystemCreateForce','MainSystemCreateTorque','MainSystemCreateCoordinateConstraint',
    'MainSystemCreateSphereSphereContact','MainSystemCreateSphereQuadContact','MainSystemCreateSphereTriangleContact',
    'MainSystemCreateFFRFReducedOrderObject',
    #plot:
    'PlotSensor', 'DataArrayFromSensorList',
    #processing:
    'ProcessParameterList', 'ParameterVariation', 'GeneticOptimization', 'Minimize', 
    #rigidBodyUtilities:
    'GetRigidBodyNode', 'AddRigidBody', #fixed problem with copy
    #robotics:
    'CreateRedundantCoordinateMBS', 'Jacobian',  'AddLidar', 'CalculateAllMeasures',
    #signal:
    'GetInterpolatedSignalValue', #checked timeArray
    #solver:
    'ComputeODE2Eigenvalues',
    #utilities:
    'ShowOnlyObjects', 'CreateDistanceSensorGeometry',
    #for several classes!:
    '__init__',
                               ]
#RaiseIssue('default args','changed several default args to None in order to remove potential problems with mutable args: interactive.InteractiveDialog(), interactive.SolutionViewer(), ...','CHANGE')

localListFunctionNames = [] #string list for highlighting
localListClassNames = [] #string list for highlighting

writeRST = True
addExampleReferences = True #costs lot of time
fileDir = paths.pythonPackageDir
filesParsed=[
             'advancedUtilities.py',
             'artificialIntelligence.py',
             'basicUtilities.py',
             'beams.py',
             'demos.py',
             'FEM.py',
             'graphics.py',
             'graphicsDataUtilities.py',
             'misc/GUI.py', 
             'misc/settingsUtilities.py',
             'interactive.py',
             'kinematicTree.py',
             'lieGroupBasics.py', #Stefan Holzinger
             'misc/mainSystemExtensions.py', 
             'particles.py',
             'physics.py',
             'plot.py',
             'processing.py',
             'rigidBodyUtilities.py',
             'robotics/roboticsCore.py',
             'robotics/rosInterface.py',
             'robotics/future.py',
             'robotics/models.py',
             'robotics/mobile.py',
             'robotics/motion.py',
             'robotics/special.py',
             'robotics/utilities.py',
             'shells.py',
             'signalProcessing.py',
             'solver.py',
             'utilities.py',
             #'lieGroupIntegration.py', #Stefan Holzinger
             ]

docuTags = ['classFunction','class','function','input','output','author','date','notes','example','status','belongsTo']
headerTags = ['Details','Author','Date','Copyright','References','Notes','Example']

argListMBSconvert = {'mbs':'self', 'mainSystem':'self'} #for conversion to class function

#function = basic/brief notes on function
#additionally, there are the following dictionary items:
#  functionName (string), 
#  argumentsList (list of strings), 
#  defaultArgumentsList (list of strings in same order as argumentsList)

def SpecialAppend(prevList, name):
    name = name.replace('\\_','_')
    if name not in prevList and name not in ['__init__', '__add__', '__iadd__', '__sub__', '__len__', '__repr__', '__getitem__', '__iter__']:
        prevList.append(name)

    return prevList

def EscapeUnderscoresOutsideMath(s):
    """Escape the underscores LaTeX would read as a subscript, and ONLY those (#2543).

    A docstring is Markdown with $...$ math (CODING_STYLE section 8), so an underscore inside $...$
    is a subscript and must stay; everywhere else it is part of a Python name - deltaL_t,
    plt.tight_layout, offsetUserFunction_t - and LaTeX answers "Missing $ inserted". The parameter
    NAME in front of a description was escaped and the description itself was not, which is how
    132 errors reached a document nobody could build.

    An underscore that already carries a backslash is left alone, so this can run over text that
    has been through ToLatex()."""
    parts = s.split('$')
    for (index, part) in enumerate(parts):
        if index % 2 == 0:                       #outside $...$; odd indices are the math spans
            parts[index] = re.sub(r'(?<!\\)_', r'\\_', part)

    return '$'.join(parts)


#convert string into latex format, reagrind _ and {}
def ToLatex(s, replaceCurlyBracket=True): #replace _ and other symbols to fit into latex code
    if replaceCurlyBracket:
        s = s.replace('{','\\{')
        s = s.replace('}','\\}')
        s = s.replace('_','\\_')

    # s = s.replace('[','\\[')
    # s = s.replace(']','\\]')
    s = s.replace('&','\\&')
    return s


def TagString2TypeAndString(tag, tagStr):
    tagType = None
    if tagStr.startswith(':'):
        if tagStr[1:].count(':') == 0:
            print('WARNING: TagString2TypeAndString: invalid tagType with one ":"')
        tagType = tagStr[1:].split(':')[0]
        tagStr = tagStr[(len(tagType)+2):]
    
    return (tagType,tagStr)

#split string with commas, but do not consider commas inside brackets or strings
#return a list of strings
def SplitStringWithCommas(s):
    strList = []
    bracket0 = 0 #( brackets
    bracket1 = 0 #[ brackets
    bracket2 = 0 #" counter (0/1)
    bracket3 = 0 #' counter (0/1)
    bracket4 = 0 #{ brackets

    currentString = ''
    for c in s:
        if c == ',' and (bracket0+bracket1+bracket2+bracket3+bracket4) == 0:
            strList += [currentString]
            #print("add string:",currentString)
            currentString= ''
        else:
            currentString += c
        if c == '(':
            bracket0 += 1
        if c == ')':
            bracket0 -= 1
        if c == '[':
            bracket1 += 1
        if c == ']':
            bracket1 -= 1
        if c == '"':
            bracket2 = 1-bracket2
        if c == "'":
            bracket3 = 1-bracket3
        if c == '{':
            bracket4 += 1
        if c == '}':
            bracket4 -= 1

    strList += [currentString.replace('{','\\{').replace('}','\\}')]
    return strList


countMutableArgs = 0
#*****************************************************
#extract function arguments for function line without leading 'def '
def GetFunctionArguments(functionLine, infoText):
    global countMutableArgs
    argumentsList = []
    defaultArgumentsList = []
    
    s = functionLine.strip()
    functionName = s.split('(')[0]
    s = s[len(functionName)+1:-1] #omit function name + '(' + ':' at end
    s = s.strip()[:-1] #omit ')' at end
    #argList = s.split(',') #does not work for default values with lists x=[1,2]
    argList = SplitStringWithCommas(s)
    for val in argList:
        val1 = val.split('=')
        argumentsList+=[val1[0].strip()]
        defaultArg = ''
        if len(val1) == 2:
            defaultArg = val1[1].strip()
            if (defaultArg.strip() != '' 
                and (defaultArg.strip() == '[]' or defaultArg.strip()[0]+defaultArg.strip()[-1] == '[]')
                and countMutableArgs < maxWarningsMutableArgs 
                and (functionName not in mutableArgsFunctionsChecked) ):
                countMutableArgs += 1
                print('potential risk ['+str(countMutableArgs)+'] with mutable function argument [] found in function:',functionName,'('+infoText+')')
                if countMutableArgs == maxWarningsMutableArgs:
                    print('  ... further WARNINGS suppressed')
        defaultArgumentsList+=[defaultArg]
        
    return [functionName,argumentsList,defaultArgumentsList]

#*****************************************************
#parse the comment header of a module (Details, Author, ...)
def _ParseModuleHeader(fileName):
    file=open(fileName,'r',encoding='utf8') 
    fileLines = file.readlines()
    file.close()

    nLines = len(fileLines)
    lineCnt = 0
    #parse header, consisting of continuous comments:
    isHeader = True #as long as comments are there, parse header
    headerDict =  {}
    tagMode = False #set true, if tag is read with subsequent lines
    headerString = ''
    currentTag = ''
    while lineCnt < nLines and isHeader:
        lineString = ToLatex(fileLines[lineCnt], replaceCurlyBracket=False) #allow latex code in header!
        if lineString[0] != '#':
            isHeader = False
            if tagMode:
                headerDict['currentTag'] = headerString
        else:
            #parse header
            tag = lineString[1:].split(':')[0].strip()
            if tag in headerTags:
                #print('tag=',tag)
                if tagMode:
                    headerDict[currentTag] = headerString
                findDS = lineString.find(':')
                headerString = lineString[findDS+1:]
                tagMode = True
                currentTag = tag
            elif (tagMode and len(lineString)>=2 and 
                  (lineString[1]==' ' or lineString[1]=='\t')):
                headerString += lineString[1:]
                #headerString += '\n'
            elif tagMode:
                headerDict[currentTag] = headerString
                tagMode = False
                
            
        lineCnt+=1 #search for identifier
    return headerDict


#*****************************************************
#the reader of the Google-style docstrings: it hands the emitters the
#dictionaries the former #** comment parser produced.
#Documented are the module-level functions and classes, and the functions of those classes, that
#have a docstring and are not marked @docmeta(public=False); author, date and status come from
#@docmeta, belongsTo from @extends.
docstringSections = {'Args': 'input', 'Returns': 'output', 'Note': 'notes', 'Example': 'example'}

def _TagValue(lines):
    """a tag value as the #** parser produced it for a tag whose text starts on the next line"""
    return '\n' + ''.join(line + '\n' for line in lines)

_citeKeys = r'[A-Za-z]+\d{4}[a-z]?(?:,\s*[A-Za-z]+\d{4}[a-z]?)*'

def Markdown2Latex(s):
    """the Markdown of the docstrings in the LaTeX flavour the emitters convert to
    .tex and RST: `code`, [Key] citations, [text](#label) references (an abbreviation if the
    label has no ':'), **bold**, *italics*, and _ escaped outside $math$ and code"""
    parts = re.split(r'(\$[^$]*\$|`[^`]*`)', s)
    for i, part in enumerate(parts):
        if part.startswith('$'):
            continue
        if part.startswith('`'):
            parts[i] = '\\texttt{' + part[1:-1].replace('_', '\\_') + '}'
            continue
        part = part.replace('%', '\\%')
        part = re.sub(r'\[Section\]\(#([^)]*)\)', r'\\refSection{\1}', part)
        part = re.sub(r'\[([^\]]*)s\]\(#\1\)', r'\\acp{\1}', part)
        part = re.sub(r'\[([^\]]*)\]\(#\1\)', r'\\ac{\1}', part)
        part = re.sub(r'\[(' + _citeKeys + r')\](?!\()', r'\\cite{\1}', part)
        part = re.sub(r'\*\*([A-Za-z][A-Za-z ]*[A-Za-z])\*\*', r'{\\bf \1}', part)
        part = re.sub(r'(?<![*\w])\*([A-Za-z]+)\*(?![*\w])', r'{\\it \1}', part)
        parts[i] = part
    return ''.join(parts)

def _DocstringItem(node, summaryTag, fileLines, fileName):
    """dictionary of one documented function or class; None if it is not documented"""
    docstring = ast.get_docstring(node, clean=False)
    if docstring is None:
        return None
    item = {}
    for decorator in node.decorator_list:
        if not isinstance(decorator, ast.Call) or not isinstance(decorator.func, ast.Name):
            continue
        if decorator.func.id == 'docmeta':
            for keyword in decorator.keywords:
                value = ast.literal_eval(keyword.value)
                if keyword.arg == 'public':
                    if not value:
                        return None
                elif value is not None:
                    item[keyword.arg] = ' ' + Markdown2Latex(value) + '\n'
        elif decorator.func.id == 'extends':
            target = decorator.args[0]
            item['belongsTo'] = target.attr if isinstance(target, ast.Attribute) else target.id

    bodyLine = fileLines[node.body[0].lineno - 1]
    indent = bodyLine[:len(bodyLine) - len(bodyLine.lstrip())]
    docLines = docstring.split('\n')
    current = summaryTag
    values = {summaryTag: [docLines[0].strip()] if docLines[0].strip() != '' else []}
    for line in docLines[1:]:
        if line.strip() == '':
            continue
        if line.rstrip()[len(indent):-1] in docstringSections and line.rstrip() == indent + line.strip():
            current = docstringSections[line.strip()[:-1]]
            values[current] = []
            continue
        prefix = indent if current == summaryTag else indent + '    '
        if not line.startswith(prefix):
            raise ValueError(fileName + ':' + str(node.lineno) + ': docstring line not indented as its '
                             + 'section requires: ' + repr(line))
        values[current].append(line[len(prefix):])
    for tag, lines in values.items():
        if tag != 'example':
            lines = [Markdown2Latex(line) for line in lines]
        if tag == summaryTag and len(lines) != 0: #the summary was written on the tag line
            item[tag] = ' ' + lines[0] + '\n' + _TagValue(lines[1:])[1:]
        else:
            item[tag] = _TagValue(lines)

    definitionLine = fileLines[node.lineno - 1].strip()
    if isinstance(node, ast.ClassDef):
        item['className'] = definitionLine[6:-1]
    else:
        functionLine = definitionLine[4:]
        lineIndex = node.lineno - 1
        while functionLine.strip()[-1] != ':':
            lineIndex += 1
            functionLine += fileLines[lineIndex] + '\n'
        [functionName, argumentsList, defaultArgumentsList] = GetFunctionArguments(functionLine, fileName)
        item['functionName'] = functionName
        item['lineNumber'] = node.lineno - 1
        item['argumentsList'] = argumentsList
        item['defaultArgumentsList'] = defaultArgumentsList
    return item

def _ParseDocstrings(fileName):
    """[functionList, classList] from the docstrings of a file"""
    source = io.open(fileName, 'r', encoding='utf8').read()
    fileLines = source.split('\n')
    functionList = []
    classList = []
    for node in ast.parse(source).body:
        if isinstance(node, ast.FunctionDef):
            item = _DocstringItem(node, 'function', fileLines, fileName)
            if item is not None:
                functionList.append(item)
        elif isinstance(node, ast.ClassDef):
            item = _DocstringItem(node, 'class', fileLines, fileName)
            if item is not None:
                item['functionList'] = [f for f in (_DocstringItem(n, 'classFunction', fileLines, fileName)
                                                    for n in node.body if isinstance(n, ast.FunctionDef))
                                        if f is not None]
                classList.append(item)
    return [functionList, classList]

def ParsePythonFile(fileName):
    """[functionList, classList, headerDict] of a utility module, from its docstrings and its comment header"""
    [functionList, classList] = _ParseDocstrings(fileName)
    return [functionList, classList, _ParseModuleHeader(fileName)]

#*****************************************************
#convert tags of tagList in functionDict to latex and RST
mycnt = 0
def FunctionStub(functionDict):
    """The .pyi overload of one MainSystem extension function: the stub half of what
    WriteFunctionDescription2LatexRST built beside the LaTeX and the RST until revision2026 step
    R7.1.7. The documentation half is FunctionDescription2Markdown below."""
    argList = functionDict['argumentsList']
    argDefault = functionDict['defaultArgumentsList']
    functionName = functionDict['functionName'].replace(chr(92) + '_', '_')

    sPyi = ' '*4 + '@overload' + chr(10)
    sPyi += ' '*4 + 'def ' + functionName + '('

    separator = ''
    for (i, argument) in enumerate(argList):
        if len(argument.strip()) == 0:
            continue
        #mbs and mainSystem become self: the function is added to the class
        modifiedArgument = argument
        for (key, value) in argListMBSconvert.items():
            modifiedArgument = modifiedArgument.replace(key, value)
        sPyi += separator + modifiedArgument.replace(chr(92) + '_', '_')
        if len(argDefault[i]) != 0:
            sPyi += '=' + argDefault[i]
        separator = ', '

    (outputType, dummy) = TagString2TypeAndString('output', functionDict['output'].strip())
    if outputType is None:
        print('missing outputType in function ', functionDict['functionName'])
        outputType = 'Any'

    sPyi += ') -> ' + outputType + ': '
    if ADD_DOCSTRINGS:
        sPyi += chr(10) + DocStringGoogleFromPlainText(
            functionDict['functionDescriptionClean'], addSpaces=' '*8) + ' '*4
    sPyi += '...' + chr(10)*2
    return sPyi


def Tags2Markdown(itemDict, tags):
    """the documented tags of a function or of a class, as Markdown;
    an example is a fenced code block, everything else a bullet"""
    text = ''
    for tag in tags:
        if tag not in itemDict or tag in ['belongsTo']:
            continue
        name = {'function': 'function description', 'class': 'class description',
                'classFunction': 'class function description'}.get(tag, tag)
        content = RemoveIndentation(itemDict[tag].strip())

        if tag == 'example':
            code = RemoveIndentation(itemDict[tag].strip(chr(10)).replace(chr(92) + '_', '_'),
                                     '  ', removeAllSpaces=False, removeIndentation=True)
            text += chr(10) + '*example*:' + chr(10) * 2 + '```python' + chr(10) \
                + code.rstrip() + chr(10) + '```' + chr(10) * 2
            continue

        if tag == 'output':
            (tagType, content) = TagString2TypeAndString(tag, content)
            if tagType is not None:
                content = '(type: ' + tagType + ')' + content

        if tag == 'input':
            arguments = ArgumentEntries(content)
            if arguments is not None:
                #one argument per line, its name as code: the Args: block of the docstring is a list
                #and was joined into one paragraph, which made a function of ten arguments one wall
                #of text (maintainer, 2026-09-26, #2665)
                text += '- **' + name + '**:' + chr(10)
                for (argument, description) in arguments:
                    text += ('  - `' + argument + '`: '
                             + LatexText2Markdown(description).replace(chr(10), ' ').strip()
                             + chr(10))
                continue

        text += '- **' + name + '**: ' + LatexText2Markdown(content).replace(chr(10), ' ') + chr(10)
    return text


#an argument of an Args: block, as the docstring writes it and Markdown2Latex left it: a name, a
#colon, and the description, which may go on over the lines that follow it
_argumentLine = re.compile(r'^([A-Za-z_][A-Za-z0-9_]*(?:\\_[A-Za-z0-9_]*)*)\s*:\s(.*)$')


def ArgumentEntries(content):
    """[(argument, description)] of an input tag that lists its arguments, or None

    None means the tag is prose and is written as it always was - an input that does not begin with
    an argument, or has none at all, is left alone rather than guessed at (#2665)."""
    entries = []
    for line in content.split(chr(10)):
        match = _argumentLine.match(line.strip())
        if match is not None:
            entries.append([match.group(1).replace(chr(92) + '_', '_'), match.group(2).strip()])
        elif len(entries) != 0 and line.strip() != '':
            entries[-1][1] += ' ' + line.strip()          #a description that went on to the next line
        elif line.strip() != '':
            return None                                  #prose before the first argument
    return [tuple(entry) for entry in entries] if len(entries) != 0 else None


def FunctionDescription2Markdown(functionDict, moduleNamePython, pythonFileName,
                                isClassFunction=False, className='', headingLevel=3,
                                labelModule=''):
    """The documentation of one function or class method, as Markdown.

    Written from the parsed dictionary rather than from the LaTeX, so that it says what it means:
    a heading with the signature, the source link, and one section per documented tag."""
    functionName = functionDict['functionName'].replace(chr(92) + '_', '_')

    lineNumber = ''
    if functionDict['lineNumber'] != 0:
        lineNumber = '#L' + str(functionDict['lineNumber'] + 1)
    url = paths.githubSourceURL + 'exudyn/' + pythonFileName + lineNumber

    arguments = []
    for (index, argument) in enumerate(functionDict['argumentsList']):
        argument = argument.strip().replace(chr(92) + '_', '_')
        if argument == '':
            continue
        default = functionDict['defaultArgumentsList'][index]
        arguments += [argument + (' = ' + default if len(default) != 0 else '')]
    signature = functionName + '(' + ', '.join(arguments) + ')'

    #the MainSystem extensions are documented under the class they are added to, not under the
    #module they live in, and their labels say so
    label = 'sec:' + (labelModule if labelModule != '' else moduleNamePython) + ':' \
        + (className + ':' if isClassFunction else '') + functionName
    text = '\n' + MarkdownLabel(label) + '\n'
    text += MarkdownHeading(('Class function: ' if isClassFunction else 'Function: ')
                            + functionName, headingLevel) + '\n\n'
    text += '[`' + signature + '`](' + url + ')\n\n'

    text += Tags2Markdown(functionDict, docuTags)
    return text + '\n'


def ModuleNames(fileName):
    """(moduleName, moduleNameLatex, moduleNamePython) of a file in filesParsed"""
    moduleName = fileName[:-3]
    moduleNameLatex = moduleName.replace('robotics/roboticsCore','robotics').replace('/','.')
    
    moduleNamePython = moduleName.split('/')[-1]
    return (moduleName, moduleNameLatex, moduleNamePython)
