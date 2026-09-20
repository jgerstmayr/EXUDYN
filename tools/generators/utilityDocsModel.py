#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  The reader of the docstrings (and @docmeta/@extends decorators) of the utility modules
#           of python/exudyn/, and the text helpers shared by mainSystemExtensionDocsEmitter.py
#           and utilityDocsEmitter.py. Moved out of src/pythonGenerator/utilitiesDocuGenerator.py
#           (revision2026 step R4.3, part 2e); reads docstrings instead of #** comments since revision2026 step R4.6.
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
from autoGenerateHelper import Str2Latex, GenerateLatexStrKeywordExamples, \
          RemoveIndentation, RSTheaderString, RSTlabelString, RSTurl, RSTmarkup, RSTcodeBlock, \
          LatexString2RST, Latex2RSTlabel, DocStringGoogleFromPlainText         # noqa: E402

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
theDocDir = paths.theDocDir
rstDir = paths.rstDir
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

def LatexString2RSTspecial(s, replaceMarkups = True): #replace \_ \{ etc. for RST

    s = s.replace('`**kwargs`','`KWARGS`')
    s = s.replace('`*args**`','`ARGS`')
    if not replaceMarkups: #don't do twice!
        s = s.replace('**kwargs','\\*\\*kwargs')
        s = s.replace('*args','\\*args')

    s = LatexString2RST(s, replaceMarkups=replaceMarkups)

    s = s.replace('`KWARGS`','`**kwargs`')
    s = s.replace('`ARGS`','`*args`')

    s = s.replace('\\ac{T66}','Plücker transformation')

    return s



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
        argumentsList+=[Str2Latex(val1[0].strip())]
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
#the reader of the Google-style docstrings (revision2026 step R4.6): it hands the emitters the
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
    """the Markdown of the docstrings (revision2026 step R4.6.4) in the LaTeX flavour the emitters convert to
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
        item['className'] = Str2Latex(definitionLine[6:-1])
    else:
        functionLine = definitionLine[4:]
        lineIndex = node.lineno - 1
        while functionLine.strip()[-1] != ':':
            lineIndex += 1
            functionLine += fileLines[lineIndex] + '\n'
        [functionName, argumentsList, defaultArgumentsList] = GetFunctionArguments(functionLine, fileName)
        item['functionName'] = Str2Latex(functionName)
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
def DictToItemsText(functionDict, tagList, addStr, eraseInput=''):
    global mycnt
    sLatex = ''
    sRST = ''
    #sIndentRST = '  '
    sSpaces = '  '*0
    for tag in tagList:
        if tag in functionDict:
            text = tag
            replaceMarkups = False
            if tag == 'function':
                text = 'function description'
                replaceMarkups = True
            if tag == 'class':
                text = 'class description'
                replaceMarkups = True
            if tag == 'input' or tag == 'classFunction' or tag == 'notes': replaceMarkups = True

            sLatex += sSpaces+'\\item[--]'+RemoveIndentation(addStr)+'{\\bf ' + text + '}: '
            sRST  += '- | '+RSTmarkup(text) + ':\n'
            # if mycnt < 10:
            #     print(RemoveIndentation(functionDict[tag].strip()), '\n') 
            #     mycnt += 1
            strTag = RemoveIndentation(functionDict[tag].strip())
            #print(strTag)
            if tag == 'output':
                (tagType,strTag) = TagString2TypeAndString(tag, strTag)
                if tagType is not None:
                    strTag = '(type: '+tagType+')'+strTag
                    #print(strTag)

            if tag == 'example':
                strTag = functionDict[tag].strip('\n').replace('\\_','_') #do not remove indentation, nor strip spaces, only blank lines
                #print("example=", strTag)
                sLatex += '\\vspace{-12pt}\\ei' #for global itemize list for function
                sLatex += '\\begin{lstlisting}[language=Python, xleftmargin=36pt]\n'
                sLatex += RemoveIndentation(strTag, '  ', removeAllSpaces = False, removeIndentation = True)
                if sLatex[-1] != '\n': sLatex+='\n'
                sLatex += '\\end{lstlisting}' #' \\vspace{6pt}'
                sLatex += '\\vspace{-24pt}\\bi\\item[]\\vspace{-24pt}' #for global itemize list for function
                sRST += '\n'+RSTcodeBlock(RemoveIndentation(strTag, '  ', removeAllSpaces = False, removeIndentation = True)+'\n', 'python')
            elif strTag.count("\n") > 0 and strTag.strip() != '': #multiple lines are replaced by list
                    
                sLatex += '\\vspace{-6pt}\n'+sSpaces+'\\begin{itemize}[leftmargin=1.2cm]\n'
                sLatex += '\\setlength{\\itemindent}{-0.7cm}\n'
                if strTag[0] == '\n':
                    strTag = strTag[1:]
                if strTag[-1] == '\n':
                    strTag = strTag[:-1]

                strTagList = strTag.split('\n')
                #replace words with ':' with italic characters
                for s in strTagList:
                    if s.strip() != '':
                        if s.find(':') != -1 and (' ' not in s[:s.find(':')]): #first occurance = argument; may not have spaces
                            n=s.find(':')
                            sr = RSTmarkup(s[:n].replace('\\_','_'),'``') + LatexString2RSTspecial(s[n:], replaceMarkups = replaceMarkups) #in this string, there should be no markup ...
                            s = '{\\it '+s[:n].replace('_','\\_')+'}'+ EscapeUnderscoresOutsideMath(s[n:])
                        else:
                            sr = LatexString2RSTspecial(s, replaceMarkups = replaceMarkups)
                            s = EscapeUnderscoresOutsideMath(s)
                        sLatex += sSpaces*2+'\\item[]'+s+'\n'
                        sRST += '  | '+RemoveIndentation(sr) + '\n'
                    
                sLatex += '\\end{itemize}\n'
                #sRST += '\n'+RemoveIndentation(strTag.strip(), '  | ')
                #sRST += '\n'
            else: #
                #sLatex += strTag.replace('\n','\\\\ \n') + '\n'
                sLatex += strTag.strip() + '\n' #in this case, we strip all spaces and newlines left, may be empty lines
                sRST += '  | ' + LatexString2RSTspecial(strTag, replaceMarkups = replaceMarkups).strip() + '\n'
    return [sLatex, sRST]

#*****************************************************
#write single function description into latex code
def WriteFunctionDescription2LatexRST(functionDict, moduleNamePython, pythonFileName, isClassFunction = False, 
                                      className='', createPyiFile=False, redirectBelongsTo=False):
    sLatex = ''
    sRST = ''
    sPyi = ''
    sPy = ''
    argList = functionDict['argumentsList']
    argDefault = functionDict['defaultArgumentsList']
    addStr = ''
    classLabelStr = ''
    if isClassFunction:
        addStr = '\\textcolor{steelblue}'
        classLabelStr = className+':'
    
    #debug:    
    # print("\n\nfunction name=",functionDict['functionName'])
    # print("\n\nfunction dict=\n",functionDict)
    functionName = functionDict['functionName']
    lineNumberStr = '' #will be e.g: '#L122'
    if functionDict['lineNumber'] != 0:
        lineNumberStr = '\\#L'+str(functionDict['lineNumber']+1)
    #github link:
    url = paths.githubSourceURL+'exudyn/'+pythonFileName +lineNumberStr


    functionNameClean = functionName.replace('\\_','_')

    if True:
        sLatex += '\\begin{flushleft}\n'
        sLatex += '\\noindent '+addStr+'{def {\\bf \\exuUrl{'+url
        sLatex += '}{' + functionName +'}{' '}}}'
    # else:
    #     sLatex += '\\noindent '+addStr+'{def '
    #     sLatex += '}{\\bf ' + functionName +'}{' '}'
    
        #relative file link:
        #sLatex += '\\noindent '+addStr+'{def \\mybold{\exuUrl{file:../../main/pythonDev/exudyn/' + moduleNamePython +'.py'+'}{' + functionName +'}{' '}}}'
        sLabel = 'sec:'
        if not createPyiFile:
            sLabel += moduleNamePython 
        else:
            sLabel += 'mainsystemextensions'
        sLabel += ':' + classLabelStr + functionNameClean
        
        if not redirectBelongsTo:
            sLatex += '\\label{'+sLabel+'}\n'
            sRST += RSTlabelString(Latex2RSTlabel(sLabel))+'\n'


    #see also https://github.com/sphinx-doc/sphinx/issues/3921
    if isClassFunction:
        title = 'Class function: '+functionNameClean
        sRST += title + '\n'
        sRST += '^'*len(title) + '\n'        
    else:
        title = 'Function: '+functionNameClean
        sRST += title + '\n'
        sRST += '^'*len(title) + '\n'    
    if True: #not createPyiFile:
        sRST += RSTurl(functionNameClean, url, False) + '_\\ (' #add another _ to make url anonymous (otherwise warning, as function name my be duplicated)
    else:
        sRST += '\\ **'+functionNameClean+'**\\ ('

    if createPyiFile:
        sPyi += ' '*4+'@overload\n'
        sPyi += ' '*4+'def '+functionNameClean+'('


    sLatex += '('
    sep = ''
    sepPyi = ''
    for i in range(len(argList)):
        argStrip = argList[i].strip()
        if len(argStrip) != 0:
            if not createPyiFile or (argStrip not in argListMBSconvert):
                sLatex += sep+'{\\it '+argList[i]+'}'
                sRST += sep + '\\ ``' + argList[i].replace('\\_','_')
                if len(argDefault[i]) != 0:
                    sLatex += '= '+argDefault[i]
                    sRST += ' = '+argDefault[i] 

                sep = ', '
                sRST += '``\\ '
            
            if createPyiFile:
                modArg = argList[i]
                for key, value in argListMBSconvert.items():
                    modArg = modArg.replace(key,value)
                sAdd = sepPyi + modArg.replace('\\_','_')
                if len(argDefault[i]) != 0:
                    sAdd += '='+argDefault[i]
                sepPyi = ', '
                
                sPyi += sAdd
                #sPy += sAdd
                #sPyReturn += sAdd

    if createPyiFile:
        output = functionDict['output'].strip()
        (outputType,dummy) = TagString2TypeAndString('output', output)
        if outputType is None:
            print('missing outputType in function ',functionDict['functionName'])
            outputType = 'Any'
        #outputType = output.split(';')[0].strip() #previous format
        #print('outputType=',outputType)
        
        sPyi += ') -> '+outputType+': '
        if ADD_DOCSTRINGS:
            sPyi += '\n' + DocStringGoogleFromPlainText(functionDict['functionDescriptionClean'], addSpaces=' '*8) + ' '*4
        sPyi += '...\n\n' #for now, we do not know the return type
        #sPyReturn += ')\n\n' 
        #sPy += '):\n'+sPyReturn
    
        functionDict = copy.deepcopy(functionDict)
        # if 'example' in functionDict:
        #     del functionDict['example']
        
        if 'input' in functionDict:
            s = functionDict['input']
            pEOL = s.find('\n',1) #start at character 1, as first character may be \n

            if not pEOL or ('mbs:' not in s[:pEOL] and 'mainSystem:' not in s[:pEOL]):
                print('ERROR: invalid input description for pyi extension')
                print(functionName)
            else:
                functionDict['input'] = functionDict['input'][pEOL+1:]
    sRST += ')\n\n'
    sLatex += ')\n'
    sLatex += '\\end{flushleft}\n'
    
    if not redirectBelongsTo:
        sLatex += '\\setlength{\\itemindent}{0.7cm}\n'
        sLatex += '\\begin{itemize}[leftmargin=0.7cm]\n'
        [sDictLatex, sDictRST] = DictToItemsText(functionDict, docuTags, addStr)
    
        sLatex += sDictLatex
        
        sRST += sDictRST
        sLatex += '\\vspace{12pt}\\end{itemize}\n%\n'

    
    
    return [sLatex,sRST,sPyi,sPy]



def Tags2Markdown(itemDict, tags):
    """the documented tags of a function or of a class, as Markdown (revision2026 step R7.1.6);
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

        text += '- **' + name + '**: ' + LatexText2Markdown(content).replace(chr(10), ' ') + chr(10)
    return text


def FunctionDescription2Markdown(functionDict, moduleNamePython, pythonFileName,
                                isClassFunction=False, className='', headingLevel=3,
                                labelModule=''):
    """The documentation of one function or class method, as Markdown (revision2026 step R7.1.6).

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
    #module they live in, and their labels say so (revision2026 step R7.1.6)
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
