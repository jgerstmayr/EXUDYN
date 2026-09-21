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
    GoogleDocstringRenderer, SplitSummaryDescription

#lists that are created during parsing
#will be used for pygments
localListFunctionNames = []
localListClassNames = []
localListEnumNames = []

#switch .pyi docstrings
ADD_DOCSTRINGS = True

#empty default argument
ArgNotSet = 'ArgNotSet'

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
#convert string to latex readable string --> used in auto-generated docu
def Str2Latex(s, isDefaultValue=False, replaceCurlyBracket=True): #replace _ and other symbols to fit into latex code

    if isDefaultValue:
        s = s.replace('true','True') #correct python notation
        s = s.replace('false','False') #correct python notation

        s = s.replace('EXUstd::InvalidIndex','invalid (-1)') #correct python notation

        if (s.find('EXUmath::unitMatrix3D') != -1): #manually done - could be automatized in future ...
            s = s.replace('EXUmath::unitMatrix3D','[[1,0,0], [0,1,0], [0,0,1]]')  
        if (s.find('EXUmath::zeroMatrix3D') != -1): #manually done - could be automatized in future ...
            s = s.replace('EXUmath::zeroMatrix3D','[[0,0,0], [0,0,0], [0,0,0]]')  

        if (s.find('Matrix6D(6,6,0.)') != -1): #manually done - could be automatized in future ...
            s = 'np.zeros((6,6))'
        
        
        if ( (s.find('Index') != -1) or (s.find('Float') != -1) or
            (s.find('Vector') != -1) or (s.find('Matrix') != -1) or 
            (s.find('Transformations66List') != -1) or (s.find('Matrix3DList') != -1) or (s.find('JointTypeList') != -1)
            ):
            s = s.replace('ArrayFloat','') #correct python notation
            s = s.replace('ArrayIndex','') #correct python notation
            s = s.replace('JointTypeList','') #KinematicTree
            s = s.replace('Vector3DList','') #KinematicTree
            s = s.replace('Vector6DList','') #KinematicTree
            s = s.replace('Vector6DList','') #KinematicTree
            s = s.replace('Matrix3DList','') #KinematicTree
            s = s.replace('Transformations66List','') #KinematicTree
            s = s.replace('Vector7D','') #correct python notation; rigid body coordinates
            s = s.replace('Vector9D','') #correct python notation; inertia parameters
            s = s.replace('Vector6D','') #correct python notation; inertia parameters
            s = s.replace('Vector4D','') #correct python notation
            s = s.replace('Vector3D','') #correct python notation
            s = s.replace('Vector2D','') #correct python notation
            s = s.replace('Vector','') #correct python notation
            s = s.replace('false','False') #correct python notation
            s = s.replace('Index2','')
            s = s.replace('Index3','')
            s = s.replace('Index4','')
            s = s.replace('Float3','')
            s = s.replace('Float4','')
            s = s.replace('Float9','')
            s = s.replace('Float16','')
            s = s.replace('EXUmath::Matrix3DFToStdArray33','')
            s = s.replace('(','[')
            s = s.replace(')',']')
            #s = s.replace('.f','')
            s = s.replace('{','')
            s = s.replace('}','')
        
        if s.find("'") == -1: #don't do that for strings!
            s = s.replace('f','')

    #s = s.replace('\\','\\\\') #leads to double \\ in latex
    s = s.replace('_','\\_')
    if replaceCurlyBracket: #don't do that for systemstructures definitions, allowing hyperlinks, etc.
        s = s.replace('{','\\{')
        s = s.replace('}','\\}')
    #s = s.replace('/',' / ')
    #s = s.replace('$','\$') #do not exclude $ in order to allow latex formulas

    return s

#parse string s and extract types available in itemType (Object/Node/...) and represent as latex-string
#possibleTypesList is e.g. Object::Body -> body 
def GetTypesStringLatex(s, itemType, possibleTypesList, separator = ','):
    returnStr = ''
    commaStr = ''
    for t in possibleTypesList:
        if s.find(itemType+'::'+t) != -1:
            #tType = t.split('::')[1] #take only left of '::'
            returnStr += commaStr+'\\texttt{'+t.replace('_','\\_')+'}'
            commaStr = separator+' '

    return returnStr

#cut the first 'numberOfCutLines' lines in a string (in order to ignore the header date in comparison of files)
def CutLinesFromString(theString, numberOfCutLines):
    pos = 0
    for i in range(numberOfCutLines):
        pos = theString.find('\n', pos) + 1

    return theString[pos:]

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
        print('WriteTextIfDifferent: illegal file: '+fileName)
        return False

    if ( (ignoreDateStrings and not IsEqualIgnoringDateStrings(fileText, text)) or
          not ignoreDateStrings and (fileText.strip() != text.strip())):
        #write file because main part has been changed
        file=open(fileName, 'w',encoding='utf8')
        file.write(text)
        file.close()
        return True
    else:
        return False

#replace '_', certain default values (e.g. Matix() --> []) and other symbols to fit into python itemInterface and for latex
def DefaultValue2Python(s): 

    s = s.replace('true','True') #correct python notation
    s = s.replace('false','False') #correct python notation

    #old, would need exu in utilities: s = s.replace('OutputVariableType::_None','OutputVariableType._None')  #this helps to avoid unreadable error messages, if type is not set; none always corresponds to 0
    s = s.replace('OutputVariableType::_None','0')  #this helps to avoid unreadable error messages, if type is not set; none always corresponds to 0
    s = s.replace('EXUmath::unitMatrix3D','IIDiagMatrix(rowsColumns=3,value=1)')  #replace with itemInterface diagonal matrix
    s = s.replace('EXUmath::zeroMatrix3D','IIDiagMatrix(rowsColumns=3,value=0)')  #replace with itemInterface diagonal matrix
    s = s.replace('Matrix()','[]')  #replace empty matrix with emtpy list
    s = s.replace('MatrixI()','[]') #replace empty matrix with emtpy list
    s = s.replace('PyMatrixContainer()','None')  #initialization in iteminterface with empty array
    s = s.replace('Vector2DList()','None')  #initialization in iteminterface with empty array
    s = s.replace('Vector3DList()','None')  #initialization in iteminterface with empty array
    s = s.replace('Vector6DList()','None')  #initialization in iteminterface with empty array
    s = s.replace('Matrix3DList()','None')  #initialization in iteminterface with empty array
    s = s.replace('BeamSectionGeometry()','exudyn.BeamSectionGeometry()')  #initialization in iteminterface with empty array
    s = s.replace('BeamSection()','exudyn.BeamSection()')  #initialization in iteminterface with empty array

    
    if (s.find('Matrix6D(6,6,') != -1):
        s = s.replace('Matrix6D(6,6,','')
        s = s.replace(')','')
        if s != '0' and s != '0.': print('error: Matrix6D(...) may only initialized with 0s')
        s = 'IIDiagMatrix(rowsColumns=6,value=' + s + ')'
        #
    elif (s.find('Matrix3D(3,3,') != -1):
        s = s.replace('Matrix3D(3,3,','')
        s = s.replace(')','')
        if s != '0' and s != '0.': print('error: Matrix3D(...) may only initialized with 0s')
        s = 'IIDiagMatrix(rowsColumns=3,value=' + s + ')'
        #
    elif ( (s.find('Index') != -1) or (s.find('Float') != -1) or 
          (s.find('Vector') != -1) or (s.find('Matrix3DList') != -1) or 
          (s.find('JointTypeList') != -1)
          ):
        s = s.replace('ArrayIndex','') #correct python notation
        s = s.replace('JointTypeList','') #KinematicTree
        # s = s.replace('Vector2DList','') #BeamSectionGeometry
        # s = s.replace('Vector3DList','') #KinematicTree
        # s = s.replace('Vector6DList','') #KinematicTree
        # s = s.replace('Matrix3DList','') #KinematicTree

        #s = s.replace('PyVector2DList','') #BeamSectionGeometry
        if s.find('PyVector2DList') != -1:
            print(s)
            raise ValueError('autoGenerateHelper(): unexpected PyVector2DList found')

        s = s.replace('Vector9D','') #correct python notation
        s = s.replace('Vector7D','') #correct python notation; rigid body coordinates
        s = s.replace('Vector6D','') #correct python notation; inertia parameters
        s = s.replace('Vector4D','') #correct python notation
        s = s.replace('Vector3D','') #correct python notation
        s = s.replace('Vector2D','') #correct python notation
        s = s.replace('Vector','') #Vector(...)-->correct python notation [...]
        s = s.replace('false','False') #correct python notation
        s = s.replace('Index2','')
        s = s.replace('Index3','')
        s = s.replace('Index4','')
        s = s.replace('Float3','')
        s = s.replace('Float4','')
        s = s.replace('Float9','')
        s = s.replace('Float16','')
        s = s.replace('EXUmath::Matrix3DFToStdArray33','')
        s = s.replace('(','[')
        s = s.replace(')',']')
        s = s.replace('f','')
        s = s.replace('{','')
        s = s.replace('}','')

    #s = s.replace('EXUstd::InvalidIndex','-1') #as we do not know the value, set it to -1; user needs to overwrite!
    #do this after replacing Index ...
    s = s.replace('EXUstd::InvalidIndex','exudyn.InvalidIndex()') #requires to import exudyn, but is possible now in itemInterface.py
    
    s = s.replace('f','')

    #not necessary in python:
    #s = s.replace('\\','\\\\')
    #s = s.replace('_','\\_') 
    #s = s.replace('{','\\{')
    #s = s.replace('}','\\}')
    #s = s.replace('$','\\$')

    return s


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
#abbreviations inside $$ latex math
convLatexMath={
    #r'\ra':     r'\rightarrow',
    # r'\LU':     r'\,^',
    #r'\Rcal':   r'\mathbb{R}',
    r'\eqDot':     r'.',
    r'\eqComma':     r',',
    r'\ImThree':     r'\mathbf{I}_{3 \times 3}',
    r'\ImTwo':     r'\mathbf{I}_{2 \times 2}',
    #r'\':     r'',
    }

abc = 'abcdefghijklmnopqrstuvwxyz'
for c in abc:
    convLatexMath['\\'+c+'v'] = r'{\mathbf{'+c+'}}'
    convLatexMath['\\'+c.upper()+'m'] = r'{\mathbf{'+c.upper()+'}}'

convLatexWords={'(\\the\\month-\\the\\year)':'',
           '    \\item':'\\item',
           '  \\item':'\\item',
           '\\item[$\\ra$]':'  |  → ', #'+ ->', #probably not used any more
           #does not work: '\\item[\\ :math:`\\ra`\\ ]':'  |  \\ :math:`\\ra`\\ ',
           #does not work: '[\\ :math:`\\ra`\\ ]':'\\ :math:`\\ra`\\ ',
           '\\item[]':'  ', 
           '\\item[--]':' - ',  #one additional whitespace at beginning for alignment of sub-lists!
           '\\item':'+ ',
           '\\finishTable':'',
           # '\\small':'', #replaced to \mysmall
           '\\noindent ':'',
           '\\noindent':'',
           '\\nonumber':'', 
           '\\phantom{XXXX}':'    ',
           '$\\ra$':'→',
           '\\textbar':'|',
           '\\lbrack':'[',
           '\\rbrack':']',
           '\\newpage':'',
           '\\tabnewline':'',
           #'\\TAB':'  ', #done in example conversion
           '\\horizontalRuler':'',
           '$\\backslash$':'\\',
           '\\plainlststyle':'',
           '\\codeName\\':'Exudyn',
           '\\codeName':'Exudyn',
           '\\pythonstyle':'',
           # '\\pythonstyle\\begin{lstlisting}':'\n.. code-block:: python\n',
           # '\\begin{lstlisting}':'\n.. code-block::\n',
           # '\\end{lstlisting}':'\n',
           '\\begin{center}':'',
           '\\end{center}':'',
           #'\\includegraphics[height=6cm]{../demo/screenshots/plotSpringDamper}':'see theDoc.pdf',
           # '+++++++++++++++++++++++++++++++':'\\ +++++++++++++++++++++++++++++++\n', #special problems with .rst
           # '=========================================':'\\ =========================================\n', #special problems with .rst
           '\\begin{itemize}':'', 
           '[leftmargin=1.4cm]':'',
           '[leftmargin=1.2cm]': '',
           '[leftmargin=0.5cm]':'', 
           '\\rule{8cm}{0.75pt}':'', 
           '\\textcolor{steelblue}':'', 
           '[language=Python, xleftmargin=36pt]':'',

           '\\bi':'', 
           '\\ei':'',
           '\\bn':'', 
           '\\en':'',
           #'\\it ':'', #replaced to \myitalics
           #specials:
           #'\\ge':'>=', #needed?
           '\\_':'_',
           '\\textdegree':'°',
           '-{}-':'--',
           #
           '{\\"a}':'ä',
           '{\\"o}':'ö',
           '{\\"u}':'ü',
           '\\"a':'ä', #if '{' is already removed earlier
           '\\"o':'ö',
           '\\"u':'ü',
           #'$':'',
           '\\rstStartNewLine':'\\ '
           }
    
#should never appear, not compatible with RST: convLabel = {'\\label':('\n\n.. _','_USE',':\n\n')} #do not do this for equation labels
convLabelEq = {'\\label':(':label: ','_USE','\n\n')} #do not do this for equation labels

convLatexCommands={#(precommand,'_USE'/'',postcommand)
    '\\ignoreRST':('','',''),
    '\\texttt':('\\ ``','_USE','``\\ '),
    #'\\label':('\n\n.. _','_USE',':\n\n'), #do this before sections ...
    '\\mysectionlabel':('','_USE','','2nd'),
    '\\mysubsectionlabel':('','_USE','','2nd'),
    '\\mysubsubsectionlabel':('','_USE','','2nd'),
    '\\mysubsubsubsectionlabel':('','_USE','','2nd'),
    '\\mysection':('','_USE',''),
    '\\mysubsection':('','_USE',''),
    '\\mysubsubsection':('','_USE',''),
    '\\mysubsubsubsection':('','_USE',''),
    #'\\pytlisting':('','',''),
    # '\\pythonSmallListing':('','',''),
    # '\\smallListing':('','',''),
    'pytlisting':('\n.. code-block:: python\n','_USE','\n'),
    'lstlisting':('\n.. code-block:: \n','_USE','\n'),
    '\\paragraph':('\n\\ **','_USE','** '),
    # '\\myListing':('','',''),
    '\\setlength':('','',''),
    '\\vspace':('','',''),
    '\\footnote':('\\ (','_USE',')'), #rst footnotes may be used instead: https://www.sphinx-doc.org/en/master/usage/restructuredtext/basics.html#footnotes
    '\\mybold':('\\ **','_USE','**\\ '),
    '\\myitalics':('\\ *','_USE','*\\ '),
    '\\mysmall':('','_USE',''), #no change of fonts for now
    #'\\mathrm':('','_USE',''),
    '\\cite':('','',''),
    '\\onlyRST':('','_USE',''),
    '\\userFunctionExample':('\n--------\n\n\\ **User function example**\\ :\n\n','',''),
    '\\userFunction':('\n--------\n\n\\ **Userfunction**\\ : ``','_USE','`` \n\n'),
    '\\LatexRSTfigure':('','_USE','','*2nd','*3rd','*4th','*5th'),

    #for tables:
    '\\startGenericTable':('\n.. list-table:: \\ \n   :widths: auto\n   :header-rows: 1\n','',''), 
    '\\rowTableThree':('','_USE','','*2nd','*3rd'),       #filled manually
    '\\rowTableFour':('','_USE','','*2nd','*3rd','*4th'), #filled manually
    '\\rowTableFive':('','_USE','','*2nd','*3rd','*4th','*5th'), #filled manually
    #
    '\\startTable':('\n.. list-table:: \\ \n   :widths: auto\n   :header-rows: 1\n','','','*2nd','*3rd'), 
    '\\rowTable':('','_USE','','*2nd','*3rd'),       #filled manually
    #'\\finishTable':('','',''),  #this is a word!
    
    '\\refSectionA':(' :ref:`Section <','_USE','>`\\ '), #anonymous -> if no header given
    '\\refSection':('Section :ref:`','_USE','`\\ '), #anonymous -> if no header given
    '\\refChapter':('Section :ref:`','_USE','`\\ '), #anonymous -> if no header given
    '\\exuUrl':('`','_USE','`_','2nd'),
    '\\url':('\\ `','_USE','`_\\ '),
    '\\ref':(' :ref:`','_USE','`\\ '),
    '\\fig':('\\ :numref:`','_USE','`\\ '), 
    #'\\fig':('Fig. :ref:`','_USE','`\\ '), 
    'figure':('','',''),
    '\\hac':('\\ :ref:`_USE <','_USE','>`\\ '),
    '\\hacs':('\\ :ref:`_USE <','_USE','>`\\ '),
    '\\acs':('\\ :ref:`_USE <','_USE','>`\\ '),
    '\\acp':('\\ :ref:`_USE <','_USE','>`\\ '),
    '\\acf':('\\ :ref:`_USE <','_USE','>`\\ '),
    '\\ac':('\\ :ref:`_USE <','_USE','>`\\ '),
    '\\eqref':('\\ :eq:`','_USE','`\\ '),
    '\\eqs':('Eqs. :eq:`','_USE','`\\ '),
    '\\eqq':('\\ :eq:`','_USE','`\\ '),
    '\\eq':('Eq. :eq:`','_USE','`\\ '),
    } #TITLE, SUBTITLE, SUBSUBTITLE, ...

#replace all occurances of conversionDict in string and return modified string
def ReplaceWords(s, conversionDict, replaceBraces=True, replaceDoubleBS=False): #replace strings provided in conversion dict

    # if replaceBraces:
        # s = s.replace('{', '')
        # s = s.replace('}', '')

    for (key,value) in conversionDict.items():
        s = s.replace(key, value)

    if replaceDoubleBS:
        s = s.replace('\\\\', '\n')

    return s

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

def Latex2RSTlabel(s):
    return s.replace(':','-').replace('_','-').lower()

#add specific markup with blind spaces
def FindMatchingBracket(s, start, openBracket='{', closingBracket='}'):
    cnt = 0
    bStart = -1
    if s[start] != openBracket:#requires to start with bracket! otherwise, this is risky!
        print('FindMatchingBracket: no bracket:',s[start-10:start+20])
        return [-1,-1]
    for i in range(start,len(s)):
        if s[i] == openBracket:
            cnt += 1
            if bStart == -1:
                bStart = i
        elif s[i] == closingBracket:
            cnt -= 1

        if bStart != -1 and cnt == 0:
            return [bStart,i]
    return [-1,-1]
        
#convert a text that is mainly designed for latex, but to be output into RST
#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the LaTeX of definitions/ and of the docstrings becomes Markdown with the same converter the
#chapters were converted with; it moved to tools/generators/latexToMarkdown.py in revision2026
#step R7.1.7, because it stopped being the one-shot tool it was written as
from latexToMarkdown import ConvertText as LatexText2Markdown                    # noqa: E402


def MarkdownLabel(latexLabel):
    """a LaTeX label as a MyST target: the same name the RST side uses, so that every reference
    that exists today keeps working"""
    return '(' + Latex2RSTlabel(latexLabel) + ')='


def MarkdownHeading(title, level):
    """level 0 is the chapter itself; the emitters count sub-sections from 1, as LaTeX does"""
    return '#' * (level + 1) + ' ' + title


def MarkdownCell(text):
    """a table cell holds no line break and no bare pipe"""
    return ' '.join(str(text).split()).replace('|', '\\|')


#a class that collects the pybind11 code, the stub text and the documentation of one
#declaration run. It was PyLatexRST and wrote LaTeX and RST beside the Markdown until revision2026
#step R7.1.7; the name and the Def... method names stay, because they are the declaration calls
#that definitions/pybind*.py is written in (pybindTypes.declarationCalls).
class PyLatexRST:
    def __init__(self, sPy='', sLatex='', sRST='', sPyi='', sMarkdown=''):
        #sLatex and sRST are accepted and ignored: the declarations pass them positionally
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
        return PyLatexRST(self.sPy+other.sPy, sMarkdown=self.sMarkdown+other.sMarkdown)

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
        self.sMarkdown += ' {ref}`' + Latex2RSTlabel(ref) + '` '

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
    def DefLatexStartTable(self, classStr='', style='', header=''):
        addInfo = ''
        if ':' in classStr:
            ni = classStr.find(':')
            addInfo = ' regarding **'+classStr[ni+1:]+'**'
            classStr = classStr[:ni]
        self.sMarkdown += ('\nThe class **' + classStr + '** has the following **functions and '
                           + 'structures**' + addInfo + ':\n\n')

    #a three column table, e.g. for output variables
    def DefLatexStartTable3(self, headers=[]):
        self.sMarkdown += ('\n| ' + ' | '.join([MarkdownCell(h) for h in headers[:3]])
                           + ' |\n|---|---|---|\n')

    #the parameter table of one item
    def DefItemStartTable(self, classStr=''):
        self.sMarkdown += ('\n| Name | type | size | default value | description |\n'
                           + '|---|---|---|---|---|\n')

    def DefLatexFinishTable(self):
        self.sMarkdown += '\n'

    def DefStartEnumClass(self, className, description, subSection=False, labelName='', cClass=None):
        if cClass==None:
            cClass = className

        self.sPy +=	'  py::enum_<' + cClass + '>(m, "' + className + '")\n'
        self.DefLatexStartClass(className, description, subSection=subSection, labelName=labelName)

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

        self.DefLatexDataAccess(itemName, description)

        self.sPyi += ' '*4 + itemName + ' = int\n' #is int correct?
        if ADD_DOCSTRINGS: self.sPyi += ' '*4 + '"""' + descriptionClean + '"""\n'

    #start a new section
    def DefLatexStartClass(self, sectionName, description, subSection=False, labelName=''):
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

        self.DefLatexStartClass(sectionName, description, subSection=subSection, labelName=labelName)

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

        self.DefLatexFinishTable()
        self.sMarkdown += '\n'

    #one data member of a class
    def DefLatexDataAccess(self, name, description, dataType = '', isTopLevel = False):
        self.sMarkdown += self.MarkdownEntry(name, description)

        if dataType != '':
            pyiIndent = ''
            if not isTopLevel:
                pyiIndent = ' '*4
            self.sPyi += pyiIndent + name + ':' + dataType+'\n'
            if ADD_DOCSTRINGS:
                self.sPyi += pyiIndent + DocStringGoogleFromPlainText(description,addSpaces='',multiline=False)

    #one operator of a class
    def DefLatexOperator(self, name, description, returnType = '',
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
        
        def ReplaceDefaultArgsLatex(s):
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
        pyNameLatex = pyName
        if pyNameLatex in pyFunctionAccessConvert:
            pyNameLatex = pyFunctionAccessConvert[pyName]
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
       
        sLadd = '  ' + Str2Latex(pyNameLatex)
        sRadd = '* | ' + '**'+Str2Latex(pyNameLatex)+'**\\ '
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
                    sLadd += ' = ' + ReplaceDefaultArgsLatex(defaultArgs[i])
                    sRadd += ' = ' + ReplaceDefaultArgsLatex(defaultArgs[i])
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
                                argString += '='+ReplaceDefaultArgsLatex(defaultArgClean)
                        sepArg = ', '
                # if pyName=='ODE1Size': #*** check if this works! check .pyi file!
                #     print(pyName+':'+argString+'; ',defaultArgs[i])
    
                self.sPyi += pyiIndent+'@overload\n'
                self.sPyi += pyiIndent+'def ' + pyName + '(' 
                self.sPyi += argString.replace('\\_','_').replace('invalid (-1)','exudyn.InvalidIndex()')
                self.sPyi += ') -> '+returnType+': '
                if ADD_DOCSTRINGS: #requires always indentation; for functions->
                    (pyiSummary,pyiDescription) = SplitSummaryDescription(description)
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
#do main part of parameter line parsing
#standard string.split() does not work, because of possible commas in description
def SplitString(string, line): #split comma separated string; commas in "..." are not counted; remove '"' and spaces outside ""

    continueOperation = True #check if parsing shall be terminated
    c = '';
    list=[]
    stringMode = 0 #0=normal mode, 1=string mode ("")
    s=''
#    for i in range[0,len(string)]:
    for c in string:
#        print('c="',c,'"')
        if continueOperation:
            if (c==',') & (stringMode != 1):
                if (stringMode != 2):
                    s = RemoveSpacesTabs(s) #to not erase interior space (e.g. initialization of vectors!) replace(' ','')
                list.append(s)
                s = ''
                stringMode = 0
            elif (c=='"'):
                if (stringMode == 0):
                    if len(s.replace(' ','').replace('\t','')) != 0:
                        print('ERROR in line',line,': invalid characters before ":',s)
                        continueOperation = False
                        list = []
                    stringMode = 1
                    s = '' #start with new string
                elif (stringMode == 1):
                    stringMode = 2 # expect comma or spaces (ignored)
            elif (stringMode != 2):
                s += c


    if (stringMode != 2):
        s = RemoveSpacesTabs(s) #to not erase interior space (e.g. initialization of vectors!) replace(' ','')
    list.append(s) #append last string; 3 commas = 4 strings   
    return list
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
    #(revision2026 step R0.2, fact 18). key=str.lower reproduces the NTFS order the
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
#writer below and by KeywordExamplesMarkdown, so that both find the same files (revision2026
#step R7.1.6). A Create* function creates the item without naming it, so it is searched too.
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


def KeywordExamplesMarkdown(itemType, itemName, itemShortName=''):
    """The examples and test models that use this item or function, as a Markdown line of links
    (revision2026 step R7.1.6). The LaTeX and RST twin below does the same for the formats that
    go away in R7.1.7; both search the same keywords and stop after the same number of files -
    an item that appears in fifty examples would otherwise push its own description off the
    page."""
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


