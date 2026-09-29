# -*- coding: utf-8 -*-
"""
Created on Fri May 18 08:53:30 2018

@author: Johannes Gerstmayr

goal: automatically generate interfaces for structures
currently: automatic generate structures with ostream and initialization
"""
import generatorPaths as paths
import datetime # for current date
import copy
import os
import re

from docstringText import CleanStringForPyiDescription, DocStringGoogleFromPlainText, RemoveIndentation,\
    GoogleDocstringRenderer, SplitSummaryDescription, StripAbbreviations, PlainTextLinks

#lists that are created during parsing
#will be used for pygments
localListFunctionNames = []
localListClassNames = []
localListEnumNames = []

#switch .pyi docstrings
ADD_DOCSTRINGS = True

#this is the list of items which will be compiled for EXUDYN_MINIMAL_COMPILATION
minimalItemsList=[
    'NodePoint',
    'ObjectGround',
    'ObjectMassPoint',
    'ObjectConnectorSpringDamper',
    'ObjectANCFCable2D', #added because needed in CContact
    'MarkerBodyPosition',
    'LoadForceVector',
    'SensorNode',
    ]

#******************************************************************************************************
def GetDateStr():
    now=datetime.datetime.now()
    monthZero = '' #add leading zero for month
    dayZero = ''   #add leading zero for day
    if now.month < 10:
        monthZero = '0'
    if now.day < 10:
        dayZero = '0'
        
    dateStr = str(now.year) + '-' + monthZero + str(now.month) + '-' + dayZero + str(now.day)

    return dateStr

#convert file name starting with lower case first letter
def FileNameLower(fileName):
    return fileName[0].lower()+fileName[1:]


#************************************************
#convert string to doxygen readable comment --> for formulas in comments and class descriptions
def Str2Doxygen(s, isDefaultValue=False): #replace _ and other symbols to fit into latex code

    s = s.replace('$','\\f$') #$ must be written as \f$ in doxygen
    s = s.replace('\\be','\\f[') #$ must be written as \f$ in doxygen
    s = s.replace('\\ee','\\f]') #$ must be written as \f$ in doxygen
    s = s.replace('\\bi','') #not needed in doxygen
    s = s.replace('\\ei','') #not needed in doxygen
    s = s.replace('\\item[]','') #not needed in doxygen
    s = s.replace('\\_','_') #not needed in doxygen

    return s

#************************************************
#the requested types of a marker or an item as documentation text, each one as a code span: it is
#written as 	exttt{...} because the text goes through LatexText2Markdown, which turns that into a
#backtick span (#2681)
#parse string s and extract types available in itemType (Object/Node/...) and represent as latex-string
#possibleTypesList is e.g. Object::Body -> body 
def GetTypesStringDocu(s, itemType, possibleTypesList, separator = ','):
    returnStr = ''
    commaStr = ''
    for t in possibleTypesList:
        if s.find(itemType+'::'+t) != -1:
            #tType = t.split('::')[1] #take only left of '::'
            returnStr += commaStr+'\\texttt{'+t.replace('_','\\_')+'}'
            commaStr = separator+' '

    return returnStr

#compare except for special date strings
#return True if files are equal, False if different
def IsEqualIgnoringDateStrings(str1, str2):
    str1list = str1.split('\n')
    str2list = str2.split('\n')
    if len(str1list) !=len(str2list):
        return False
    for i in range(len(str1list)):
        if str1list[i].startswith('* @date') or str1list[i].startswith('// AUTO:  last modified'):
            continue
        if str1list[i] != str2list[i]:
            #print('diff="'+str1list[i]+'" vs. "'+str2list[i]+'"')
            return False

    return True

# write string 'text' to fileName if it differs from existing content of fileName; 
# if ignoreDateStrings=True, then ignore lines with dates (as they may change always!)
# return True if file has been written (and changed), otherwise False
def WriteTextIfDifferent(fileName, text, ignoreDateStrings):
    if os.path.isfile(fileName):
        file=open(fileName,'r',encoding='utf8')
        fileText = file.read()
        file.close()
    else:
        #A FILE THAT DOES NOT EXIST IS WRITTEN, not refused (#2647). This used to print 'illegal
        #file' and return, which is how a generator could silently write nothing for years: on
        #linux the output of structureHeaderEmitter differed from the tracked file by one capital
        #letter, so it took this branch on every run. A generator that declares an output has to
        #be able to create it - generate.py checks afterwards that every declared output is there.
        fileText = None

    if (fileText is None or
        (ignoreDateStrings and not IsEqualIgnoringDateStrings(fileText, text)) or
          not ignoreDateStrings and (fileText.strip() != text.strip())):
        #write file because main part has been changed
        file=open(fileName, 'w',encoding='utf8')
        file.write(text)
        file.close()
        return True
    else:
        return False

#************************************************
# helper function for reading the structure
def RemoveSpacesTabs(s):
    s = s.replace('\t','')
    s = s.strip(' ') #to not erase interior space (e.g. initialization of vectors!) replace(' ','')

    return s


#************************************************
#count lines to see if changes effect the number of written lines
def CountLines(s):
    location = -1
    strLen = len(s)
    counter = 1 #first line does not have a linebreak!

    while location < strLen:
        location = s.find('\n', location + 1)
        if location == -1: 
            location = strLen
        else:
            counter += 1
    return counter

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

def MarkdownLabelName(s):
    """the name a section label is referenced by: lower case, and ':' and '_' as '-'"""
    return s.replace(':','-').replace('_','-').lower()

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the LaTeX of definitions/ and of the docstrings becomes Markdown with the same converter the
#chapters were converted with; it is tools/generators/latexToMarkdown.py, a module of the generators
#rather than the one-shot tool it was written as
from latexToMarkdown import ConvertText as LatexText2Markdown                    # noqa: E402


def MarkdownLabel(label):
    """a LaTeX label as a MyST target: the same name the RST side uses, so that every reference
    that exists today keeps working"""
    return '(' + MarkdownLabelName(label) + ')='


def MarkdownHeading(title, level):
    """level 0 is the chapter itself; the emitters count sub-sections from 1, as LaTeX does"""
    return '#' * (level + 1) + ' ' + title


def MarkdownCell(text):
    """a table cell holds no line break and no bare pipe"""
    return ' '.join(str(text).split()).replace('|', '\\|')


#a class that collects the pybind11 code, the stub text and the documentation of one
#declaration run: the three outputs it writes. Its Def... methods are the declaration calls that
#definitions/pybind*.py is written in, and pybindTypes.declarationCalls lists them by name, so a
#method renamed here is renamed there (#2681)
class DeclarationWriter:
    def __init__(self, sPy='', sPyi='', sMarkdown=''):
        self.sPy = sPy
        self.sPyi = sPyi
        self.sMarkdown = sMarkdown
        self.markdownPages = []      #(pageName, text), filled by CreateNewPage
        self.currentPageName = ''

    def Reset(self):
        self.sPy = ''
        self.sPyi = ''
        self.sMarkdown = ''
        self.markdownPages = []
        self.currentPageName = ''

    def __add__(self, other):
        return DeclarationWriter(self.sPy+other.sPy, sMarkdown=self.sMarkdown+other.sMarkdown)

    def __iadd__(self, other):
        self = self + other
        return self

    def PyStr(self): return self.sPy

    def PyAdd(self, s):
        self.sPy += s

    def MarkdownStr(self): return self.sMarkdown

    def MarkdownAdd(self, s):
        self.sMarkdown += s

    #close the current documentation page and start a new one; '' only closes
    def CreateNewPage(self, pageName):
        if self.currentPageName != '':
            self.markdownPages += [(self.currentPageName, self.sMarkdown)]
            self.sMarkdown = ''
        self.currentPageName = pageName

    #the declarations still call this by its old name
    def CreateNewRSTfile(self, fileName):
        self.CreateNewPage(fileName)

    #add text for documentation; labels in LaTeX have ':' as separator, MyST targets have '-'
    def AddDocu(self, text, section='', sectionLevel=1, sectionLabel='', preNewLine=True):
        if section != '':
            #a blank line first - a target glued to the paragraph above it is not a target - then
            #the target, so that a {ref} to it finds a heading with a title
            if not self.sMarkdown.endswith('\n\n'):
                self.sMarkdown += ('\n' if self.sMarkdown.endswith('\n')
                                   else '\n\n')
            if sectionLabel != '':
                self.sMarkdown += MarkdownLabel(sectionLabel) + '\n'
            self.sMarkdown += MarkdownHeading(section, sectionLevel) + '\n\n'

        if len(text) != 0 and text.strip(' ')[-1] != '\n':
            text += '\n'

        #a paragraph of its own: two AddDocu calls in a row are two paragraphs, not one
        markdownText = LatexText2Markdown(text)
        if markdownText != '':
            if self.sMarkdown != '' and not self.sMarkdown.endswith('\n\n'):
                self.sMarkdown += '\n' if self.sMarkdown.endswith('\n') else '\n\n'
            self.sMarkdown += markdownText + '\n'

    #one entry of what LaTeX drew as a table row: a function, a data member or an operator
    def MarkdownEntry(self, signature, description, example=''):
        text = '- **`' + signature.strip() + '`**'
        description = LatexText2Markdown(description).replace('\n', ' ').strip()
        if description != '':
            text += ': ' + description
        text += '\n'

        if example != '':
            code = example.replace('\\\\', '\n').replace('\\#', '#').replace('\\TAB', '  ')
            code = RemoveIndentation(code, '', False).strip('\n')
            #two spaces, so that the fenced block belongs to the list item above it
            text += '\n  *Example*:\n\n  ```python\n'
            for line in code.split('\n'):
                text += ('  ' + line).rstrip() + '\n'
            text += '  ```\n\n'
        return text

    def AddInlineRef(self, ref):
        self.sMarkdown += ' {ref}`' + MarkdownLabelName(ref) + '` '

    def AddDocuCodeBlock(self, code, pythonStyle=True, addRSTLineNumbers=True):
        if code.strip(' ')[-1] != '\n':
            code += '\n'
        self.sMarkdown += ('\n```' + 'python'*pythonStyle + '\n'
                           + RemoveIndentation(code, '', False).strip('\n') + '\n```\n\n')

    def AddDocuList(self, itemList, itemText=''):
        if len(itemList) != 0:
            for item in itemList:
                self.sMarkdown += '- ' + LatexText2Markdown(item).replace('\n', ' ').strip() + '\n'
            self.sMarkdown += '\n'

    #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #for the pybind interface documentation:

    #the sentence that introduces the functions and structures of a class
    def DefStartTable(self, classStr='', style='', header=''):
        addInfo = ''
        if ':' in classStr:
            ni = classStr.find(':')
            addInfo = ' regarding **'+classStr[ni+1:]+'**'
            classStr = classStr[:ni]
        self.sMarkdown += ('\nThe class **' + classStr + '** has the following **functions and '
                           + 'structures**' + addInfo + ':\n\n')

    #a three column table, e.g. for output variables
    def DefStartTable3(self, headers=[]):
        self.sMarkdown += ('\n| ' + ' | '.join([MarkdownCell(h) for h in headers[:3]])
                           + ' |\n|---|---|---|\n')

    #the parameter table of one item
    def DefItemStartTable(self, classStr=''):
        self.sMarkdown += ('\n| Name | type | size | default value | description |\n'
                           + '|---|---|---|---|---|\n')

    def DefFinishTable(self):
        self.sMarkdown += '\n'

    def DefStartEnumClass(self, className, description, subSection=False, labelName='', cClass=None):
        if cClass==None:
            cClass = className

        self.sPy +=	'  py::enum_<' + cClass + '>(m, "' + className + '")\n'
        self.DefStartClass(className, description, subSection=subSection, labelName=labelName)

        self.sPyi += '\nclass '+className+'(Enum):\n'
        self.sPyi += DocStringGoogleFromPlainText(description, addSpaces='    ', multiline=True,
                                                  splitSummaryDescription=True)

    #add an enum value to the pybind interface and to the documentation
    def AddEnumValue(self, className, itemName, description):
        descriptionClean = CleanStringForPyiDescription(description)
        self.sPy += '		.value("' + itemName + '", ' + className + '::' + itemName
        self.sPy +=  ', "' + descriptionClean + '"' #if ADD_DOCSTRING could be added
        self.sPy +=  ')\n'

        if className not in localListEnumNames:
            localListEnumNames.append(className)

        self.DefDataAccess(itemName, description)

        self.sPyi += ' '*4 + itemName + ' = int\n' #is int correct?
        if ADD_DOCSTRINGS: self.sPyi += ' '*4 + '"""' + descriptionClean + '"""\n'

    #start a new section
    def DefStartClass(self, sectionName, description, subSection=False, labelName=''):
        self.sMarkdown += '\n'
        if labelName != '':
            self.sMarkdown += MarkdownLabel(labelName) + '\n'
        self.sMarkdown += MarkdownHeading(LatexText2Markdown(sectionName), 1+1*subSection) + '\n\n'
        self.sMarkdown += LatexText2Markdown(description) + '\n\n'

    #start class definition
    def DefPyStartClass(self, cClass, pyClass, description, subSection = False, labelName='',
                        forbidPythonConstructor = False):
        if pyClass != '' and pyClass not in localListClassNames:
            localListClassNames.append(pyClass)

        self.sPy += '\n'
        sectionName = pyClass
        if (cClass == ''):
            sectionName = '\\codeName' #for EXUDYN, work around

        if (cClass != ''):
            self.sPy += '    py::class_<' + cClass + '>(m, "' + pyClass + '")\n'
            if not forbidPythonConstructor:
                self.sPy += '        .def(py::init<>())\n'
            else:
                self.sPy += '        .def(py::init(&'+cClass+'::ForbidConstructor))\n'

        self.DefStartClass(sectionName, description, subSection=subSection, labelName=labelName)

        classInfo = 'exudyn module'
        if pyClass != '': #in case of basic module, stubs are not needed => information
            classInfo = 'class '+pyClass

        self.sPyi += '\n#stub information for '+classInfo+' functions\n'
        if pyClass != '': #in case of basic module, stubs are not needed => information
            self.sPyi += 'class ' + pyClass + ':\n'
            if ADD_DOCSTRINGS and len(description.strip()):
                self.sPyi += DocStringGoogleFromPlainText(description,addSpaces=' '*4,
                                                          multiline=True, splitSummaryDescription=True)

    def DefPyFinishClass(self, cClass):
        if (cClass != ''):
            self.sPy += '        ; // end of ' + cClass + ' pybind definitions\n\n'

        self.DefFinishTable()
        self.sMarkdown += '\n'

    #one data member of a class
    def DefDataAccess(self, name, description, dataType = '', isTopLevel = False, readOnly = False):
        self.sMarkdown += self.MarkdownEntry(name, description)

        if dataType != '':
            pyiIndent = ''
            if not isTopLevel:
                pyiIndent = ' '*4
            if readOnly and not isTopLevel:
                #a member that can be read and not written is a property, and the stub says so (#2541)
                self.sPyi += pyiIndent + '@property\n'
                self.sPyi += pyiIndent + 'def ' + name + '(self) -> ' + dataType + ':\n'
                if ADD_DOCSTRINGS:
                    self.sPyi += pyiIndent + ' '*4 + DocStringGoogleFromPlainText(description,addSpaces='',multiline=False)
                self.sPyi += pyiIndent + ' '*4 + '...\n'
                return
            self.sPyi += pyiIndent + name + ':' + dataType+'\n'
            if ADD_DOCSTRINGS:
                self.sPyi += pyiIndent + DocStringGoogleFromPlainText(description,addSpaces='',multiline=False)

    #one operator of a class
    def DefOperator(self, name, description, returnType = '',
                         argList=[], defaultArgs=[], argTypes=[],
                         isTopLevel = False):
        hasArgs = bool(len(argList))
        if len(defaultArgs):
            argStr = (', '.join([f'{key}={value}' for key, value in zip(argList, defaultArgs)]) )*hasArgs
        else:
            argStr = (', '.join(argList) )*hasArgs

        self.sMarkdown += self.MarkdownEntry('operator ' + name + '(' + argStr + ')', description)

        if returnType != '':
            pyiIndent = ''
            if not isTopLevel:
                pyiIndent = ' '*4
            argStr = (', '+', '.join([f'{key}: {value}' for key, value in zip(argList, argTypes)]))*hasArgs
            self.sPyi += pyiIndent+'@overload\n'
            self.sPyi += pyiIndent+'def ' + name + '(self'+argStr+') -> ' + returnType+': ...\n'

    #************************************************
    #helper functions to create manual pybinding to access functions in classes
    #pyName = python name, cName=full path of function in C++, description= textual description used in C and in documentation
    #argList = [arg1Name, arg2Name, ...]
    #defaultArgs = [arg1Default, ...]: either empty or must have same length as argList
    #options= additional manual options (e.g. memory model)
    #example = string, which is put into latex documentation
    #isLambdaFunction = True: cName is intepreted as lambda function and copied into pybind definition
    def DefPyFunctionAccess(self, cClass, pyName, cName, description, argList=[], defaultArgs=[], 
                            example='', options='', isLambdaFunction = False, 
                            argTypes=[], returnType = '', addDocu=True): 
        
        if pyName not in localListFunctionNames:
            localListFunctionNames.append(pyName)

        
        def ReplaceDefaultArgsCpp(s):
            sNew = copy.copy(s)
            sNew = sNew.replace('exu.','') #remove exudyn 'exu.' for C-code
            sNew = sNew.replace('True','true').replace('False','false') #docu shows True, C++ code needs true
            return sNew
        
        def ReplaceDefaultArgsDocu(s):
            sNew = copy.copy(s)
            sNew = sNew.replace('EXUstd::InvalidIndex','invalid (-1)') #if changed: check other places for "invalid (-1)"
            sNew = sNew.replace('Contact::IndexEndOfEnumList','ContactTypeIndex.IndexEndOfEnumList') #if changed: check other places for "invalid (-1)"
            sNew = sNew.replace('true','True').replace('false','False')
            if sNew.find('Vector3D') != -1:
                sNew = sNew.replace('(std::vector<Real>)Vector3D','')
                sNew = sNew.replace('{','').replace('}','')
                sNew = sNew.replace('(','[').replace(')',']')
            sNew = sNew.replace('py::','').replace('::','.') #replace C-style '::' (e.g. in ConfiguationType) to python-style '.'            
            return sNew
        
        #make some checks:
        if (len(argList) != 0) & (len(defaultArgs) == 0):
            defaultArgs = ['']*len(argList)
        elif len(argList) != len(defaultArgs):
            print('error in command '+pyName+': defaultArgs are inconsistent')
            return ''
        
        if (cClass != ''):
            self.sPy += '        .def("'
        else:
            self.sPy += '        m.def("'
    
        #convert some special functions, like __repr__()
        addBraces = True
        pyNameDocu = pyName
        if pyNameDocu in pyFunctionAccessConvert:
            pyNameDocu = pyFunctionAccessConvert[pyName]
            addBraces = False
            #print('now pyName=', pyName)
    
        self.sPy += pyName + '", ' 
        if not(isLambdaFunction): #if lambda function ==> just copy cName as code
            self.sPy += '&' 
            if (cClass != ''):
                self.sPy += cClass + '::'
    
        self.sPy += cName + ', '
        self.sPy += '"' + description +'"'
        if (options != ''):
            self.sPy += ', ' + options
       
        sLadd = '  ' + pyNameDocu
        sRadd = '* | ' + '**'+pyNameDocu+'**\\ '
        if addBraces: 
            sLadd += '('
            sRadd += '('
        if len(argList):
            sSep = ''
            for i in range(len(argList)):
                if argList[i] != '*args': #won't work in pybind interface (see comment in Pybind11 docs)
                    self.sPy += ', py::arg("' + argList[i] + '")'

                sLadd += sSep+argList[i]
                sRadd += sSep+'\\ *'+argList[i].replace('*','\\*')+'*\\ '
                if (defaultArgs[i] != ''):
                    if argList[i] != '*args': #won't work in pybind interface (see comment in Pybind11 docs)
                        self.sPy += ' = ' + ReplaceDefaultArgsCpp(defaultArgs[i])
                    sLadd += ' = ' + ReplaceDefaultArgsDocu(defaultArgs[i])
                    sRadd += ' = ' + ReplaceDefaultArgsDocu(defaultArgs[i])
                sSep = ', '

        self.sPy += ')'
                
        if (cClass == ''):
            self.sPy += ';'
        
        self.sPy += '\n'

        examplePyi = '' #the stub block below runs whether or not the function is documented
        if addDocu:
            if example != '':
                #the example goes into the stub docstring as well, with the two-space TAB
                examplePyi = example.replace('\\\\', '\n').replace('\\#', '#').replace('\\TAB', '  ')

            signature = sLadd.strip().replace('\\_', '_') + ')'*addBraces
            self.sMarkdown += self.MarkdownEntry(signature, description, example=example)
    
        #the stub describes what the module HAS; addDocu only decides whether the function
        #is DOCUMENTED. Every deprecated function carries addDocu=False and was therefore
        #missing from the .pyi, so a type checker reported its use in user code as an error
        #(#2490). The example text stays with the documentation.
        if '.' not in pyName: #'special.InfoStat' and friends are not module-level names
            pyiIndent = ''
            if cClass != '': #in case of basic module, stubs are not needed => information 
                pyiIndent = ' '*4
            if returnType != '':
                hasTypes = (len(argTypes) == len(argList)) and (len(argList) != 0)
                # if len(argTypes) != len(argList):
                #     raise ValueError('DefPyFunctionAccess: inconsistent argList / argTypes')
    
                argString = 'self'*(cClass!='')
                if len(argList):
                    sepArg = ', '*(argString!='')
                    for i in range(len(argList)):
                        argString += sepArg + argList[i]
                        if hasTypes and argTypes[i]!='':
                            argString += ': '+argTypes[i]
                        if i < len(defaultArgs):
                            defaultArgClean = defaultArgs[i].replace('exu.','')
                            if defaultArgClean != '':
                                argString += '='+ReplaceDefaultArgsDocu(defaultArgClean)
                        sepArg = ', '
                # if pyName=='ODE1Size': #*** check if this works! check .pyi file!
                #     print(pyName+':'+argString+'; ',defaultArgs[i])
    
                self.sPyi += pyiIndent+'@overload\n'
                self.sPyi += pyiIndent+'def ' + pyName + '(' 
                self.sPyi += argString.replace('\\_','_').replace('invalid (-1)','exudyn.InvalidIndex()')
                self.sPyi += ') -> '+returnType+': '
                if ADD_DOCSTRINGS: #requires always indentation; for functions->
                    (pyiSummary,pyiDescription) = SplitSummaryDescription(StripAbbreviations(PlainTextLinks(description)))
                    data = {
                    "kind": "function",
                    "summary": pyiSummary,
                    "description": pyiDescription,
                    }
                    if examplePyi!='':
                        data['examples'] = [examplePyi]
                    #"notes": ["asdf."],
                    # "inputs": [
                    #     {"name": "mbs", "description": "The MainSystem where items are created.", "type_hint": None},
                    #     {"name": "name", "description": "Name string for the object."},
                    #     {"name": "physicsMass"},
                    # ],
                    # "output": {"type_hint": "Union[dict, ObjectIndex]", "description": "Object index or a dict if returnDict=True."},
                    # "output": {"description": pyiDescription},
                    # "examples": ["MainSystemCreateMassPoint(mbs, physicsMass=1.0)"],
                    renderer = GoogleDocstringRenderer()
                    self.sPyi += '\n' #newline
                    self.sPyi += renderer.render(data, indent=pyiIndent+ ' '*4)
                    self.sPyi += '\n' + pyiIndent+ ' '*4

                self.sPyi += '...\n'


    #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #for SystemStructures:
        
    #one row for definition of system structures
    def SystemStructuresWriteDefRow(self, pythonName, typeName, sSize, sDefaultVal, description, typicalPaths = [], isFunction=False):
        #the name cell carries the FULL access path where there is one -
        #'SC.visualizationSettings.general.autoFitScene' is what a user types
        markdownName = pythonName + ('(...)' if isFunction and sDefaultVal != ''
                                     else '()' if isFunction else '')
        nameCell = '`' + MarkdownCell(markdownName) + '`'
        for path in typicalPaths:
            separator = '.' if path != '' else ''
            nameCell += '<br>`' + MarkdownCell(path + separator + pythonName) + '`'
        self.sMarkdown += ('| ' + nameCell + ' | ' + MarkdownCell(typeName)
                           + ' | ' + MarkdownCell(sSize) + ' | '
                           + (MarkdownCell(sDefaultVal) if sDefaultVal != '' else '')
                           + ' | ' + MarkdownCell(LatexText2Markdown(description)) + ' |\n')

    #one row of the parameter table of an item
    def ItemInterfaceWriteRow(self, pythonName, typeName, sSize='', sDefaultVal='', sSymbol='', description=''):
        #\tabnewline is a LaTeX line break for column widths and means nothing here
        def Cell(content):
            return MarkdownCell(LatexText2Markdown(content.replace('\\tabnewline', ' ')))

        nameCell = '**' + pythonName + '**'
        if sSymbol.strip() != '':
            nameCell += ' $' + sSymbol.strip().strip('$') + '$'
        self.sMarkdown += ('| ' + nameCell + ' | ' + Cell(typeName) + ' | ' + Cell(sSize)
                           + ' | ' + Cell(sDefaultVal) + ' | ' + Cell(description) + ' |\n')

    #one row for a three column table, e.g. for output variables
    def Table3WriteRow(self, cols=['','',''], typeList=['','',''], nameLiteral=True):
        typeCell = cols[1] if typeList[1] == '' else typeList[1] + ', ' + cols[1]
        self.sMarkdown += ('| ' + MarkdownCell(cols[0]) + ' | '
                           + MarkdownCell(LatexText2Markdown(typeCell)) + ' | '
                           + MarkdownCell(LatexText2Markdown(cols[2])) + ' |\n')


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#convert type to known C++ type or keep it (in case of special class)
def GenerateHeader(classStr, descriptionStr, addModifiedDate = True, addIfdefOnce = True, 
                   author = ''):

    now=datetime.datetime.now()
    monthZero = '' #add leading zero for month
    dayZero = ''   #add leading zero for day
    hourZero= ''
    minuteZero= ''
    secondZero= ''
    
    if now.month < 10:
        monthZero = '0'
    if now.day < 10:
        dayZero = '0'
    if now.hour < 10:
        hourZero = '0'
    if now.minute < 10:
        minuteZero = '0'
    if now.second < 10:
        secondZero = '0'
        
    dateStr = str(now.year) + '-' + monthZero + str(now.month) + '-' + dayZero + str(now.day)
    timeStr = hourZero+str(now.hour) + ':' + minuteZero+str(now.minute) + ':' + secondZero+str(now.second)
    
    if author == '':
        author = 'Gerstmayr Johannes'
    
    #************************************
    #header
    s='' #generate a string
    #s+='//automatically generated file (pythonAutoGenerateInterfaces.py)\n'
    s+='/** ***********************************************************************************************\n'
    s+='* @class        '+classStr+'\n'
    s+='* @brief        '+descriptionStr+'\n'
    s+='*\n'
    s+='* @author       '+author+'\n'
    s+='* @date         2019-07-01 (generated)\n'
    if addModifiedDate:
        s+='* @date         '+ dateStr+ '  ' + timeStr + ' (last modified)\n' #this causes all files to change ...
    #s+='* @date         2019-09-12 (last modfied)\n'
    s+='*\n'
    s+='* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.\n'
    s+='* @note         Bug reports, support and further information:\n'
    s+='                - email: johannes.gerstmayr@uibk.ac.at\n'
    s+='                - weblink: https://github.com/jgerstmayr/EXUDYN\n'
    s+='                \n'
    s+='************************************************************************************************ */\n'
    
    if addIfdefOnce:
        #only works for MSVC:
        #        s+='#ifdef _MSC_VER\n'
        #        s+='#pragma once\n'
        #        s+='#endif\n'
        s+='\n'
        s+='#ifndef '+classStr.upper()+'__H\n'
        s+='#define '+classStr.upper()+'__H\n'
    s+='\n'
    s+='#include <ostream>\n'
    s+='\n'
    s+='#include "Utilities/ReleaseAssert.h"\n'
    s+='#include "Utilities/BasicDefinitions.h"\n'
    s+='#include "System/ItemIndices.h"\n'
    s+='\n'

    return s


#************************************************
pyFunctionAccessConvert = {
    '__repr__': '__repr__()',
    '__getitem__': '... = data[index]',
    '__setitem__': 'data[index]= ...',
    '__len__': 'len(data)',
    }


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the following functions were originally in pythonAutoGenerateObjects.py
    
#get list of filenames in folder dirPath which contain keyword (to find examples with specific items)
#checkPreString: check in particular, if as letter 'a-zA-Z' or '0-9'
def ExtractExamplesWithKeyword(keyword, dirPath, checkPreString=True):
    from os import listdir
    from os.path import isfile, join
    
    #sorted(): listdir() returns DIRECTORY order, which is alphabetical on NTFS but hash
    #order on ext4. Without this the generated output differs between Windows and Linux
    #for no real reason - found 2026-09-11 by the first CI run of tools/regenerate.py
    #. key=str.lower reproduces the NTFS order the
    #committed output was generated in, so making this deterministic did not also
    #reshuffle every documentation file.
    fileNames = sorted([f for f in listdir(dirPath) if isfile(join(dirPath, f))],
                      key=str.lower)
    
    filesWithKeyword = []

    for fileName in fileNames:
        if fileName[-3:]=='.py':
            #print("extract example:",fileName)
            #file = open(dirPath+'/'+fileName)
            with open(dirPath+'/'+fileName, mode='r', encoding='utf-8') as file:
                text = file.read()

            keywordPos = text.find(keyword)
            found = keywordPos
            if checkPreString and found > 0: #exclude keywords that do not fully match (e.g. MassPoint and CreateMassPoint)
                if (text[found-1].isalpha() or text[found-1].isdecimal()):
                    #print('found invalid example for "'+keyword+'": '+text[max(0,found-10):(found+len(keyword))])
                    found = -1
            if found != -1:
                filesWithKeyword += [fileName]
            file.close()
    return filesWithKeyword

#generate latex string containing a list of file references (and hyperref links), 
#based on a search through Examples and TestModels
#if latex is false, formatting is clean to be used in RST
#the keywords under which the examples of an item are searched; shared by the LaTeX/RST
#writer below and by KeywordExamplesMarkdown, so that both find the same files. A Create*
#function creates the item without naming it, so it is searched too.
createFunctionOfItem = {
    'ObjectFFRF': 'AddObjectFFRF(',
    'ObjectFFRFreducedOrder': 'AddObjectFFRFreducedOrderWithUserFunctions(',
    'ObjectRigidBody': 'CreateRigidBody(',
    'NodeRigidBodyEP': 'CreateRigidBody(',
    'ObjectConnectorSpringDamper': 'CreateSpringDamper(',
    'ObjectConnectorCartesianSpringDamper': 'CreateCartesianSpringDamper(',
    'ObjectConnectorRigidBodySpringDamper': 'CreateRigidBodySpringDamper(',
    'ObjectConnectorTorsionalSpringDamper': 'CreateTorsionalSpringDamper(',
    'ObjectJointRevoluteZ': 'CreateRevoluteJoint(',
    'ObjectPrismaticJointX': 'CreatePrismaticJoint(',
    'ObjectJointSpherical': 'CreateSphericalJoint(',
    'ObjectJointGeneric': 'CreateGenericJoint(',
    'ObjectConnectorDistance': 'CreateDistanceConstraint(',
    'ObjectConnectorCoordinate': 'CreateCoordinateConstraint(',
    'ObjectJointRollingDisc': 'CreateRollingDisc(',
    'ObjectConnectorRollingDiscPenalty': 'CreateRollingDiscPenalty(',
    #mbs. avoids the ambiguity with robot.CreateKinematicTree
    'ObjectKinematicTree': 'mbs.CreateKinematicTree(',
    'LoadForceVector': 'CreateForce(',
    'LoadTorqueVector': 'CreateTorque(',
    }


def ExampleKeywords(itemType, itemName, itemShortName=''):
    """what to search the Examples and TestModels folders for"""
    if itemType == 'UtilityFunction':
        return [itemName + '(']

    keywords = ['mbs.Add' + itemType + '(' + itemName + '(']
    if itemName in createFunctionOfItem:
        keywords += [createFunctionOfItem[itemName]]
    if itemShortName != '' and itemName != itemShortName:
        keywords += ['mbs.Add' + itemType + '(' + itemShortName + '(']
    return keywords


def KeywordExamplesMarkdown(itemType, itemName, itemShortName='', examples=None):
    """The examples and test models that use this item or function, as a Markdown line of links.
    It stops after a fixed number of files - an item that appears in fifty examples would otherwise
    push its own description off the page. examples, paths relative to python/, replaces the search
    by the ones the definition chose (#2737)."""
    if examples is not None:
        links = ['[`' + name.split('/')[-1] + '`](' + paths.githubSourceURL + name + ')'
                 + (' (Ex)' if name.startswith('Examples/') else ' (TM)') for name in examples]
        return ('\nExamples (Ex) and TestModels (TM) that show this item, with weblink to github: '
                + ', '.join(links) + '\n\n') if len(links) != 0 else ''
    keywords = ExampleKeywords(itemType, itemName, itemShortName)
    maxExamples = 5 if itemType == 'UtilityFunction' else 12

    links = []
    truncated = False
    for (folderIndex, folder) in enumerate(['Examples', 'TestModels']):
        found = []
        for keyword in keywords:
            for name in ExtractExamplesWithKeyword(keyword=keyword,
                                                   dirPath=paths.pythonDir + folder):
                if name not in found:
                    found += [name]

        abbreviation = ' (Ex)' if folder == 'Examples' else ' (TM)'
        for name in found:
            #the cap runs over both folders, as in the LaTeX and RST writer
            if len(links) >= maxExamples + 3*folderIndex:
                truncated = True
                break
            links += ['[`' + name + '`](' + paths.githubSourceURL + folder + '/' + name + ')'
                      + abbreviation]

    if len(links) == 0:
        return ''
    return ('\nRelevant Examples (Ex) and TestModels (TM) with weblink to github: '
            + ', '.join(links) + (', ...' if truncated else '') + '\n\n')


