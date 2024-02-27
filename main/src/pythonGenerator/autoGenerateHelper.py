# -*- coding: utf-8 -*-
"""
Created on Fri May 18 08:53:30 2018

@author: Johannes Gerstmayr

goal: automatically generate interfaces for structures
currently: automatic generate structures with ostream and initialization
"""
import datetime # for current date
import copy

#lists that are created during parsing
#will be used for pygments
localListFunctionNames = []
localListClassNames = []
localListEnumNames = []

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

#************************************************
#convert type to known C++ type or keep it (in case of special class)
def TypeConversion(typeStr, typeConversion):
    newStr = typeStr
    if (typeStr in typeConversion):
        newStr = typeConversion[typeStr]
        
    #print('convert "'+typeStr+'" into "'+newStr+'"')

    return newStr

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
           '\\ge':'>=',
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
    '\\myListing':('','',''),
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
def RSTmarkup(name, c='*', blindSpaces=True):
    return '\\ '*blindSpaces+c+name+c+'\\ '*blindSpaces


#add code block; code must already be indented; code must have \n at end
def RSTcodeBlock(code, typeString='', addLineNumbers=False, indentation=''):
    s = '.. code-block:: '+typeString + '\n'
    icode = code

    if indentation != '':
        icode = RemoveIndentation(code,indentation, False, False) #add 3 spaces

    if addLineNumbers:
        if indentation == '':
            s += '   '
        s += indentation + ':linenos:\n'

    s += '\n' + icode + '\n'
    return s

#create text for inline URL
def RSTurl(urlText, link, boldFace=False, blindSpaces=False):
    bf = ''
    if boldFace:
        bf = ':stlink:'
    return '\\ '*blindSpaces+bf+'`'+urlText+' <'+link+'>`_'+'\\ '*blindSpaces


#write latex source inline
def RSTinlineMath(mathString, convert=True):
    if convert:
        mathString = ReplaceWords(mathString, convLatexMath, replaceBraces=False) #keep braces!

    s = '\\ :math:`' + mathString + '`\\ '
    return s

#equation block:
def RSTmathEq(mathString, convert=True, isEqArray=False):
    if convert:
        mathString = ReplaceWords(mathString, convLatexMath, replaceBraces=False, replaceDoubleBS=False) #keep braces!
    
    #single line, line break would indicate separate math line
    sNew = '.. math::\n'

    #+++++++++++++++++++++++++++++++
    s = mathString
    found = ExtractLatexCommand(s, '\\label', False, False)

    label = ''
    if found != -1:
        [preString, innerString, innerString2, postString] = found
        s = preString + postString
        label = Latex2RSTlabel(innerString)
        #print('label found:', label)

    #+++++++++++++++++++++++++++++++
    #we need to remove spaces at beginning/end and remove lines with comments and empty lines
    slines = s.split('\n')
    s = ''
    for line in slines:
        line = line.strip()
        if len(line) == 0 or line[0] == '%':
            continue #empty line omitted
        s+=line+'\n'
    if len(s) == 0:
        raise ValueError('RSTmathEq: empty equation:'+mathString)
    s = s[:-1]
    s = RemoveIndentation(s).replace('\n',' ') #must be single line; spaces kept, as they are needed as delimiter

    if isEqArray:
        s = s.replace('\\\\','\\\\'+'\n')
    s = RemoveIndentation(s, addSpaces='   ')

    if label != '':
        sNew += '   :label: '+label+'\n\n'
    else:
        sNew += '\n'

    sNew += s + '\n\n'
    # if isEqArray:
    #     print('math=\n',sNew)
    return sNew
    
#write latex source in separate equation
def RSTmath(mathString, label=''):
    s = '.. math:: '+mathString+'\n'
    if label!='':
        s+='   :label: '+label + '\n'
    s += '\n'
    return s

#label string directly to be placed e.g. before header
def RSTlabelString(name):
    return '\n.. _'+Latex2RSTlabel(name)+':\n'


#writer heder with levels from 1 to 4
def RSTheaderString(header, level):
    s = ''
    if level == 0:
        s += '='*len(header)+'\n'
        s += header+'\n'
        s += '='*len(header)+'\n'
    elif level == 1:
        s += '*'*len(header)+'\n'
        s += header+'\n'
        s += '*'*len(header)+'\n'
    elif level == 2:
        #s += '-'*len(header)+'\n'
        s += header+'\n'
        s += '='*len(header)+'\n'
    elif level == 3:
        s += header+'\n'
        s += '-'*len(header)+'\n'
    elif level == 4:
        s += header+'\n'
        s += '^'*len(header)+'\n'
    else:
        raise ValueError('WriteRSTheader: unknown header level: '+str(level))
    # print('header=\n',s)
    return s

#start searching for { and matching } bracket, including sub-brackets
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
def LatexString2RST(s0, replaceCommands=True, replaceMarkups = False, sectionMarkerText=''): #Latex style to RST

    # replace latex math to RST inline
    sNew = ''
    endFound = False
    pos = 0

    s = s0
    s0 = s0.replace(r'\pythonstyle\begin{lstlisting}',r'\begin{pytlisting}')

    if replaceCommands:
        s = ReplaceLatexCommands(s0, convLatexCommands, sectionMarkerText=sectionMarkerText)

    while not endFound:
        startStr = '$'
        endStr = '$'
            
        isInlineEq = True
        val = s.find(startStr,pos)
        val2 = s.find('\\be',pos)
        if val2 != -1 and (val2 < val or val == -1): #cases: val=-1, val2=-1 || val=-1, val2>=0 || val>=0, val2=-1 || val>-1, val2>-1; 
            isInlineEq = False
            startStr = '\\be'
            endStr = '\\ee'
            val = val2

        isEqArray = False
        if val == -1:
            val = len(s)
            endFound = True
        else: #do not consider \begin{} or \bea !
            if not isInlineEq:
                #print('found be:', s[val:val+30])
                sep = s[val+len(startStr)] 
                if sep == 'a': #eqnarray
                    startStr = '\\bea'
                    endStr = '\\eea'
                    sep = s[val+len(startStr)]
                    isEqArray = True
                if sep.strip() != '' and sep != '\\':
                    #print('not')
                    pos = val+len(startStr)
                    continue #continue search at pos
            

        regText = s[pos:val]
        if replaceMarkups:
            regText = regText.replace('*','\\*')
            
        if replaceCommands:
            #commands must be replaced already earlier, e.g. rowTable may include $$ inside, this would not work if split up into parts
            # regText = ReplaceLatexCommands(regText, convLatexCommands)
            regText = ReplaceWords(regText , convLatexWords, replaceBraces=True, replaceDoubleBS=True)

        sNew += regText
        if not endFound:
            val += len(startStr)
            valEnd = s.find(endStr,val)
            if valEnd != -1:
                if isInlineEq:
                    sMath = s[val:valEnd].strip() #spaces at end make problems
                    sNew += RSTinlineMath(sMath) #do not replace markups inside this text
                else:
                    sNew += '\n' + RSTmathEq(s[val:valEnd],True, isEqArray=isEqArray)
                    
                val = valEnd+len(endStr)
            else:
                print('WARNING:\nutilities: found no closing '+endStr+' in:\n', s)
                endFound = True
                sNew = s
        pos = val

    #

    if '\\label' in sNew:
        print('WARNING: label still in s:')
        sf = sNew.find('\\label')
        print(sNew[sf:sf+40])
    # if '\\be' in sNew:
    #     print('WARNING: \\be still in s:')
    #     sf = sNew.find('\\be')
    #     print(sNew[sf:sf+40])

    return sNew

#if key is found, return [preString, innerString, innerString2, postString], otherwise -1; '123\section{abc}456' = ['123','abc','456']
#if secondBracket>0, it searches two consecutive brackets: \exuURL{...}{...} and stores in innerString2 (will be list for more inner strings)
def ExtractLatexCommand(s, key, secondBracket, isBeginEnd=False):
    found = -1
    if isBeginEnd: #find \begin{...} \end{...}
        #always find next occurances --> will be erased in next run ...
        keyEnd = '\\end{'+key+'}'
        keyStart = '\\begin{'+key+'}'
        if key == 'pytlisting':
            keyEnd = '\\end{'+'lstlisting'+'}'
        if key == '\\be':
            keyStart = key
            keyEnd = '\\ee'
        sStart = s.find(keyStart)
        sEnd = s.find(keyEnd,sStart)
        if sStart == -1 or sEnd == -1 or sStart >= sEnd: #if start>end, it is e.g. lstlisting end for pytlisting as start
            # if sStart >= sEnd:
            #     print('begin/end: sStart>sEnd: key=',key)
            return -1
        else:
            found = sStart
            sStart += len(keyStart) - 1
            if False:
                print('=====================')
                print('found:'+s[found:sEnd+len(keyEnd)])
                print('+++++++++++++++++++++')
                print('inner:'+s[sStart:sEnd])
                print('=====================')
            #sEnd += len('\\end{'+key+'}') - 1
            preString = s[:found]
            innerString = s[sStart+1:sEnd]
            postString = s[sEnd+len(keyEnd):]

            return [preString, innerString, '', postString]
    else:
        found = s.find(key)
        while found != -1 and len(s) > found+len(key) and s[found+len(key)] != '{':
            found = s.find(key, found+1)
        
        if found != -1:
            [sStart, sEnd] = FindMatchingBracket(s, found+len(key))

            preString = s[:found]
            if sEnd == -1:
                print('no matching bracket found: '+key+', "'+s[found:min(found+20,len(s))]+'"')
                raise ValueError('ERROR')
    
            innerString = s[sStart+1:sEnd]
            postString = s[sEnd+1:]
    
            # sStart2 = -1
            # sEnd2 = -1
            innerString2 = ''
            innerStringList = []
            
            if secondBracket > 0:
                for k in range(int(secondBracket)):
                    #print('find 2nd: '+s[sEnd+1:sEnd+80])
                    [sStart2, sEnd2] = FindMatchingBracket(s, sEnd+1)
                    if sEnd2 == -1:
                        print('no matching second bracket found: '+key+', "'+s[sEnd+1:sEnd+50]+'"')
                        raise ValueError('ERROR')
                    innerString2 = s[sStart2+1:sEnd2]
                    innerStringList += [innerString2]
                    postString = s[sEnd2+1:]
                    sEnd = sEnd2
                if secondBracket > 1: #for more than 2 arguments
                    innerString2 = [innerString]+innerStringList
                    #print('innerString2:',innerString2)
    
            # print(sStart, sEnd)
            return [preString, innerString, innerString2, postString]
        else:
            return -1

def ReplaceLatexCommands(s, conversionDict, sectionMarkerText=''): #replace strings provided in conversion dict
    sectionFilesDepth = 1
    secOff = 0
    if sectionMarkerText!='':
        secOff = 1#for doc2rst the section headers are different
        
    #remove comments:
    slines = s.split('\n')
    s = ''
    for line in slines:
        fc = line.find('%')
        if fc != -1 and len(line) >= fc and line[fc-1] != '\\': #\% should be kept; line with \% should not have comment ...
            if len(line[:fc].lstrip()) != 0:
                line = line[:fc] #remove everything behind that; keep %, to be consistent with following steps

        ls = line.lstrip()
        if len(ls) != 0 and ls[0] == '%': #emtpy lines are kept, %lines are removed!
            continue
        #in rowtable, the new table may not contain any extra spaces at beginning of lines
        if line.find('\\rowTable') != -1:
            line = line.lstrip()

        s+=line+'\n'
    s = s[:-1]

    #remove commands
    s = s.replace('{\\bf ','\\mybold{') #this is then further converted into rst code ...
    s = s.replace('{\\it ','\\myitalics{') #this is then further converted into rst code ...
    s = s.replace('{\\it ','\\mysmall{') #this is then further converted into rst code ...
    for (key,value) in conversionDict.items():
        found = 0
        while (found != -1):
            isBeginEnd = False
            secondBracket = 0
            if len(value) > 3:
                secondBracket = len(value)-3
            # if key == '\\exuUrl' or 'sectionlabel' in key: 
            #     secondBracket = True
            if (key == 'figure' 
                or key == 'lstlisting' 
                or key == 'pytlisting'
                or key == '\\be'
                ):
                isBeginEnd = True
            
            found = ExtractLatexCommand(s, key, secondBracket, isBeginEnd)

            if found != -1:
                [preString, innerString, innerString2, postString] = found
                s = preString
                if ('\\refSection' in key or key == '\\label' or key == '\\fig' or key == '\\ref'
                    or key == '\\eq' or key == '\\eqref' or key == '\\eqs' or key == '\\eqq'):
                    innerString=Latex2RSTlabel(innerString)
                elif (value[1] == '_USE' and key != '\\exuUrl' and key != '\\url' 
                      and ('\\ac' not in key) 
                      and key != '\\onlyRST' and key != '\\footnote' #final replacement in exterior loop!
                      and key != 'lstlisting' and key!='pytlisting'):
                    if 'lstlisting' in innerString:
                        print('WARNING: lstlisting in inner string:',innerString)
                    innerString = ReplaceLatexCommands(innerString, convLatexCommands)
                    innerString = ReplaceWords(innerString, convLatexWords) #needs to be cleaned here already

                # if ('lstlisting' in key):
                #     print('==============\n'+preString[-20:]+value[0]+innerString + postString[:40])

                if '\\rowTable' in key or key=='\\startTable':
                    #nRows = len(value)-2
                    if key=='\\startTable':
                        s += value[0] + '\n'
                    #print('rowTableThree/Four: rows=',nRows)
                    if type(innerString2) == list:
                        # if len(innerString2) != nRows:
                        #     print('innerString2:',innerString2)
                        text = ''
                        cstar = '*'
                        for k, col in enumerate(innerString2):
                            text += '   '+cstar+' - | '+col
                            if k < len(innerString2)-1:
                                text += '\n' #last \n is added due to text itself (postString)
                            cstar = ' '
                        s += text
                        #print('table = \n'+text)
                    else:
                        print('PROBLEM with rowTable: ',innerString2)
                elif '\\LatexRSTfigure' in key:
                    # print(innerString2)
                    text =  '\n\n.. _'+Latex2RSTlabel(innerString2[1])+':\n'
                    text += '.. figure:: docs/theDoc/'+innerString2[0]+'.png\n'
                    text += '   :width: '+innerString2[3]+'\n'
                    text += '\n'+'   '+LatexString2RST(innerString2[4]) + '\n\n'
                    s += text
                    # print('figure:\n'+text)
                elif key == '\\mysection' or key == '\\mysectionlabel':
                    if sectionMarkerText != '':
                        s += sectionMarkerText+'[0]'+'[' + innerString + ']'+'\n'
                    if 'label' in key: 
                        s += RSTlabelString(innerString2)+'\n'
                    #sectionsList += [('0',innerString)]
                    s += '\n'+RSTheaderString(innerString, secOff + 0)
                elif key == '\\mysubsection' or key == '\\mysubsectionlabel':
                    if sectionFilesDepth > 0 and sectionMarkerText != '':
                        s += sectionMarkerText+'[1]'+'[' + innerString + ']'+'\n'
                        #sectionsList += [('1',innerString)]
                    if 'label' in key: s += RSTlabelString(innerString2)+'\n'
                    s += '\n'+RSTheaderString(innerString, secOff + 1)
                # elif key == '\\mysection' or key == '\\mysectionlabel':
                #     if 'label' in key: s += RSTlabelString(innerString2)+'\n'
                #     s += '\n'+RSTheaderString(innerString, secOff + 0)
                # elif key == '\\mysubsection' or key == '\\mysubsectionlabel':
                #     if 'label' in key: s += RSTlabelString(innerString2)+'\n'
                #     s += '\n'+RSTheaderString(innerString, secOff + 1)
                elif key == '\\mysubsubsection' or key == '\\mysubsubsectionlabel':
                    if 'label' in key: s += RSTlabelString(innerString2)+'\n'
                    s += '\n'+RSTheaderString(innerString, secOff + 2)
                elif key == '\\mysubsubsubsection' or key == '\\mysubsubsubsectionlabel':
                    if 'label' in key: s += RSTlabelString(innerString2)+'\n'
                    s += '\n'+RSTheaderString(innerString, secOff + 3)
                elif key == '\\exuUrl' or key == '\\url':
                    if key == '\\url': innerString2=innerString
                    s += value[0]
                    s += innerString2 + ' <' + innerString + '>'
                    s += value[2]
                else:
                    if '_USE' in value[0]:
                        s += value[0].replace('_USE', innerString)
                    else:
                        s += value[0]
                    if value[1] == '_USE':
                        s += innerString
                    s += value[2]
                
                s += postString
                
    return s

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#a class that handles triples of strings: Pybind, Latex, RST
class PyLatexRST:
    #initialize strings
    def __init__(self, sPy='', sLatex='', sRST='', sPyi=''):
        
        self.sPy = sPy
        self.sLatex = sLatex
        self.sRST = sRST
        self.sPyi = sPyi
        self.rstFileLists = [] #contains tuples (filename, text)
        self.rstCurrentFileName = '' #if this is non-empty, it will be stored in list with current text

    def Reset(self): 
        self.sPy = ''
        self.sLatex = ''
        self.sRST = ''
        self.rstFileLists = [] #contains tuples (filename, text)
        self.rstCurrentFileName = '' #if this is non-empty, it will be stored in list with current text
        
    def __add__(self, other):
        return PyLatexRST(self.sPy+other.sPy,self.sLatex+other.sLatex,self.sRST+other.sRST)

    #x += PyLatexRST('a','b','c')
    def __iadd__(self, other):
        self = self + other
        return self

    def GetStringsList(self):
        return [self.sPy, self.sLatex, self.sRST]

    def PyStr(self): return self.sPy
    def LatexStr(self): return self.sLatex
    def RSTStr(self): return self.sRST

    def PyAdd(self, s):
        self.sPy += s

    def LatexAdd(self, s):
        self.sLatex += s

    def RSTAdd(self, s):
        self.sRST += s

    #start new file in 
    def CreateNewRSTfile(self, fileName):
        if self.rstCurrentFileName != '':
            self.rstFileLists += [(self.rstCurrentFileName, self.sRST)]
            self.sRST = '' #start new text
        self.rstCurrentFileName = fileName
    

    #add text for documentation, 0=chapter
    #labels in latex have ':' as separator, in RST have '-'
    def AddDocu(self, text, section='', sectionLevel=1, sectionLabel='', preNewLine = True):
        if preNewLine:
            self.sLatex += '\n'
            self.sRST += '\n'
        if section != '':
            self.sLatex += '\\my'+'sub'*sectionLevel + 'section{' + section + '}\n'

            if not preNewLine:  #always needed for section label and heading
                self.sRST += '\n'
            if sectionLabel != '':
                self.sLatex += '\\label{'+sectionLabel+'}\n'
                self.sRST += RSTlabelString(sectionLabel)+'\n'

            self.sRST += RSTheaderString(section, sectionLevel)
            self.sRST += '\n'
        if len(text) != 0 and text.strip(' ')[-1] != '\n':
            text += '\n'

        self.sLatex += text
        self.sRST += LatexString2RST(text)

    #add inline reference in latex format, converted to RST: latex labels have ':' as separator, in RST have '-'
    def AddInlineRef(self, ref):
        self.sLatex += '\\refSection{'+ref+'}'
        self.sRST += ' :ref:`'+Latex2RSTlabel(ref)+'`\\ '

        
    #add python style code blocks to latex and RST
    #line numbers in Latex not preferable because copying does not work well; line numbers only if code>3lines
    def AddDocuCodeBlock(self, code, pythonStyle=True, addRSTLineNumbers=True):
        # print('code0=', ord(code[0]))
        if code.strip(' ')[-1] != '\n':
            code += '\n'
        self.sLatex +='\\pythonstyle\n'
        self.sLatex +='\\begin{lstlisting}[language=Python, firstnumber=1]\n'
        self.sLatex += code
        self.sLatex +='\\end{lstlisting}\n\n'

        spaces='   '
        self.sRST += '\n.. code-block:: ' + 'python'*pythonStyle + '\n' #needs empty line in between
        if addRSTLineNumbers and code.count('\n') > 4:
            self.sRST += spaces+':linenos:\n'
        self.sRST += '\n'*(code.strip(' ')[0] != '\n')
        self.sRST += RemoveIndentation(code, spaces,False)
        #print(RemoveIndentation(code, '   '))
        self.sRST += '\n'*(code.strip(' ')[-1] != '\n')
        

    #add python style code blocks to latex and RST
    def AddDocuList(self, itemList, itemText=''):
        if len(itemList) != 0:
            self.sLatex += '\\bi\n'
            self.sRST += '\n'
            for item in itemList:
                sEnd = '\n'*(item.strip(' ')[-1] != '\n') #add separator if not there already
                self.sLatex +='  \\item'+itemText+' ' + item + sEnd
                if itemText == '':
                    rstItem = '*'
                elif itemText == '[]':
                    rstItem = ' '
                else: 
                    print('WARNING: AddDocuList: illegal itemText:'+itemText)
                
                self.sRST += rstItem + RemoveIndentation(LatexString2RST(item), '  | ')[1:] + sEnd
    
            self.sRST += '\n'
            self.sLatex += '\\ei'

    #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #for autoGEneratePyBindings:
        
    #start a new table to describe class bindings in latex;
    def DefLatexStartTable(self, classStr='', style='| p{8cm} | p{8cm} |',
                           header = '{\\bf function/structure name} & {\\bf description}'):
        self.sLatex += '\\begin{center}\n'
        self.sLatex += '\\footnotesize\n'
        self.sLatex += '\\begin{longtable}{'+style+'} \n'
        self.sLatex += '\\hline\n'
        self.sLatex += header+'\\\\ \\hline\n'

        #self.sRST += '\n\ **Description of functions and structures**:\n\n'
        addInfo = ''
        if ':' in classStr:
            ni = classStr.find(':')
            addInfo = ' regarding **'+classStr[ni+1:]+'**'
            classStr = classStr[:ni]
        self.sRST += '\n\ The class **'+classStr+'** has the following **functions and structures**'+addInfo+':\n\n'

    #start a new table to describe class bindings in latex;
    def DefLatexStartTable3(self, headers=[]):

        self.sLatex += '\\begin{center}\n'
        self.sLatex += '\\footnotesize\n'
        self.sLatex += '\\begin{longtable}{| p{5cm} | p{5cm} | p{6cm} |} \n'
        self.sLatex += '\\hline\n'
        self.sLatex += '\\bf '+headers[0]+' & \\bf '+headers[1]+' & \\bf '+headers[2]+' \\\\ \\hline\n'

        self.sRST += '\n'

    #start a new table to describe class bindings in latex;
    def DefItemStartTable(self, classStr=''):
        sTemp   = '%reference manual TABLE\n'
        sTemp  += '\\begin{center}\n'
        sTemp  += '  \\footnotesize\n'
        sTemp  += '  \\begin{longtable}{| p{4.5cm} | p{2.5cm} | p{0.5cm} | p{2.5cm} | p{6cm} |}\n'
        sTemp  += ' '*4+'\\hline\n'
        sTemp  += ' '*4+'\\bf Name & \\bf type & \\bf size & \\bf default value & \\bf description \\\\ \\hline\n'
        self.sLatex += sTemp

        self.sRST += '\n'

    #finish latex table for class bindings 
    def DefLatexFinishTable(self):
        self.sLatex += '\\end{longtable}\n'
        self.sLatex += '\\end{center}\n'
        self.sRST += '\n\n' #empty line closes list block

    def DefStartEnumClass(self, className, description, subSection=False, labelName='', cClass=None):
        if cClass==None:
            cClass = className
            
        self.sPy +=	'  py::enum_<' + cClass + '>(m, "' + className + '")\n'
        self.DefLatexStartClass(className, description, subSection=subSection, labelName=labelName)
        
        self.sPyi += '\nclass '+className+'(Enum):\n'

    #add a enum value and definition to pybind interface and to latex documentation
    def AddEnumValue(self, className, itemName, description):
        self.sPy += '		.value("' + itemName + '", ' + className + '::' + itemName + ')    //' + description + '\n'

        #this function is for enums
        if className not in localListEnumNames:
            localListEnumNames.append(className)

        #self.sLatex += '  ' + Str2Latex(itemName) + ' & ' + Str2Latex(description) + '\\\\ \\hline \n'
        self.DefLatexDataAccess(itemName, description) #Str2Latex(...) done inside function

        self.sPyi += ' '*4 + itemName + ' = int\n' #is int correct?

    #start a new section
    def DefLatexStartClass(self, sectionName, description, subSection=False, labelName=''):
    
        self.sLatex +=  "\n%++++++++++++++++++++\n"
        if subSection:
            self.sLatex += "\\mysubsubsection"
        else:
            self.sLatex += "\\mysubsection"
    
        self.sLatex += "{" + sectionName + "}\n"
        if labelName != '':
            self.sLatex += '\\label{' +labelName+ '}\n'
            self.sRST += RSTlabelString(labelName) + '\n'
            
        self.sLatex += description + '\n\n'
    
        self.sRST += '\n'+RSTheaderString(LatexString2RST(sectionName), 1+1*subSection) + '\n'
        self.sRST += RemoveIndentation(LatexString2RST(description)) + '\n' #empty line needed for list
    
    #start class definition
    def DefPyStartClass(self, cClass, pyClass, description, subSection = False, labelName='', 
                        forbidPythonConstructor = False):
        if pyClass != '' and pyClass not in localListClassNames:
            localListClassNames.append(pyClass)

        self.sPy += '\n'
        sectionName = pyClass
        if (cClass == ''): 
            #print("ERROR::DefPyStartClass: cClass must be a string")
            sectionName = '\\codeName' #for EXUDYN, work around
        
        if (cClass != ''):
            self.sPy += '    py::class_<' + cClass + '>(m, "' + pyClass + '")\n'
            if not forbidPythonConstructor:
                self.sPy += '        .def(py::init<>())\n'
            else:
                # constructorCode = 'CHECKandTHROWstring("'+pyClass+'() may not be called from Python. '+forbidPythonConstructorStr+'");'            
                # self.sPy += '        .def(py::init([]() { '+constructorCode+' } )) //!< AUTO: forbid constructor call from Python\n'
                self.sPy += '        .def(py::init(&'+cClass+'::ForbidConstructor))\n'
    
        self.DefLatexStartClass(sectionName, description, subSection=subSection, labelName=labelName)

        classInfo = 'exudyn module'
        if pyClass != '': #in case of basic module, stubs are not needed => information 
            classInfo = 'class '+pyClass

        self.sPyi += '\n#stub information for '+classInfo+' functions\n'
        if pyClass != '': #in case of basic module, stubs are not needed => information 
            self.sPyi += 'class ' + pyClass + ':\n'
        
    
    def DefPyFinishClass(self, cClass):
        
        if (cClass != ''):
            self.sPy += '        ; // end of ' + cClass + ' pybind definitions\n\n'
    
        self.DefLatexFinishTable()
        self.sRST += '\n'

    #add latex table entry / RST list entry for data variable
    def DefLatexDataAccess(self, name, description, dataType = '', isTopLevel = False): 
        self.sLatex += '  ' + Str2Latex(name) + ' & '+Str2Latex(description) + '\\\\ \\hline  \n'
        self.sRST += '* | ' + '**'+Str2Latex(name)+'**:\n'
        self.sRST += RemoveIndentation(LatexString2RST(description), '  | ') + '\n'
        
        if dataType != '':
            if not isTopLevel:
                self.sPyi += ' '*4
            self.sPyi += name + ':' + dataType+'\n'
            
    #add latex table entry / RST list entry for data variable
    def DefLatexOperator(self, name, description, returnType = '', 
                         argList=[], defaultArgs=[], argTypes=[], 
                         isTopLevel = False): 
        hasArgs = bool(len(argList))
        if len(defaultArgs):
            argStr = (', '.join([f'{key}={value}' for key, value in zip(argList, defaultArgs)]) )*hasArgs
        else:
            argStr = (', '.join(argList) )*hasArgs
        
        self.sLatex += '  operator ' + Str2Latex(name) + '('+argStr+') & '+Str2Latex(description) + '\\\\ \\hline  \n'
        self.sRST += '* | operator ' + '**'+Str2Latex(name)+'**\ ('+argStr+'):\n'
        self.sRST += RemoveIndentation(LatexString2RST(description), '  | ') + '\n'
        
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
                            argTypes=[], returnType = ''): 
        
        if pyName not in localListFunctionNames:
            localListFunctionNames.append(pyName)

        
        def ReplaceDefaultArgsCpp(s):
            sNew = copy.copy(s)
            sNew = sNew.replace('exu.','') #remove exudyn 'exu.' for C-code
            sNew = sNew.replace('True','true').replace('False','false') #docu shows True, C++ code needs true
            return sNew
        
        def ReplaceDefaultArgsLatex(s):
            sNew = copy.copy(s)
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

        self.sLatex += sLadd
        self.sRST += sRadd
        
        if addBraces: 
            self.sLatex += ')'
            self.sRST += ')'
    
        self.sPy += ')'
                
        if (cClass == ''):
            self.sPy += ';'
        
        self.sPy += '\n'

        self.sLatex += ' & ' + description.replace('_','\\_')
        #self.sRST += ': \n' +  RemoveIndentation(description.replace('_','\_'), '  | ') + '\n'
        self.sRST += ': \n' +  RemoveIndentation(LatexString2RST(description), '  | ') + '\n'
        if example != '':
            exampleRST = example
            example = Str2Latex(example)
            example = example.replace('\\\\','\\tabnewline\n    ').replace('#','\\#')

            example = example.replace('\\TAB','\\phantom{XXXX}') #phantom spaces, not visible
            self.sLatex += '\\tabnewline \n    \\textcolor{steelblue}{{\\bf EXAMPLE}: \\tabnewline \n    \\texttt{' + example.replace("'","{\\textquotesingle}") + '}}'
            exampleRST = exampleRST.replace('\\\\','\n').replace('\\#','#')
            
            self.sRST += '  | *Example*:\n\n'
            self.sRST += '  '+RSTcodeBlock(RemoveIndentation(exampleRST,'   '+'  ', False).replace('\\TAB','  '), 'python') + '\n' #TAB=2 spaces +2 spaces surrounding
        self.sLatex += '\\\\ \\hline \n'

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
                    elif i < len(defaultArgs):
                        argString += '='+ReplaceDefaultArgsLatex(defaultArgs[i].replace('exu.',''))
                    sepArg = ', '

            self.sPyi += pyiIndent+'@overload\n'
            self.sPyi += pyiIndent+'def ' + pyName + '(' + argString + ') -> '+returnType+': ...\n'


    #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #for SystemStructures:
        
    #one row for definition of system structures
    def SystemStructuresWriteDefRow(self, pythonName, typeName, sSize, sDefaultVal, description, typicalPaths = [], isFunction=False):
        #latex:
        typeNameLatex = typeName
        if len(pythonName)>28:  #for space of pythonname over column width
            typeNameLatex = '\\tabnewline ' + typeName

        latexFuncStr = ''
        if isFunction:
            if sDefaultVal == '':
                latexFuncStr = '()'
            else:
                latexFuncStr = '(...)'

        self.sLatex += '    ' + pythonName+latexFuncStr + ' & '
        self.sLatex += '    ' + typeNameLatex + ' & '
        self.sLatex += '    ' + sSize + ' & '
        self.sLatex += '    ' + sDefaultVal + ' & '
        self.sLatex += '    ' + description + '\\\\ \\hline\n' #Str2Latex not used, must be latex compatible!!!
        

        #RST:
        argStr = '('*isFunction
        if sDefaultVal != '' and isFunction:
            #s += ', default = ' + sDefaultVal
            argStr += sDefaultVal
        argStr += ')'*isFunction
            
        s = '* | **' + pythonName+argStr + '** [' + 'return '*isFunction + 'type = ' + typeName
        if sDefaultVal != '' and not isFunction:
            s += ', default = ' + sDefaultVal
        if sSize != '':
            s += ', size = '+sSize
        s += ']:\n'
        if typicalPaths != []:
            s += '  | '
            sep = ''
            for p in typicalPaths:
                pdot = ''
                if p != '':
                    pdot = p+ '.'
                s += sep + RSTmarkup(pdot + pythonName, c='``') 
                sep = ', '
            s += '\n'
            
        s += RemoveIndentation(LatexString2RST(description), '  | ') + '\n'
        self.sRST += s
        
    #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #for Item Interfaces
        
    #one row for definition of items 
    def ItemInterfaceWriteRow(self, pythonName, typeName, sSize='', sDefaultVal='', sSymbol='', description=''):
        self.sLatex += ' '*4 + Str2Latex(pythonName) + ' & '                
        self.sLatex += ' '*4 + typeName + ' & '
        self.sLatex += ' '*4 + sSize + ' & '
        self.sLatex += ' '*4 + sDefaultVal + ' & '
        self.sLatex += ' '*4 + description + '\\\\ \\hline\n' #Str2Latex not used, must be latex compatible!!!
        
        #RST:
        s = '* | **' + pythonName + '** ['
        
        if sSymbol.strip() != '':
            s += RSTinlineMath(sSymbol.strip('$')) + ', '

        s += 'type = ' + typeName

        sDefaultVal = LatexString2RST(sDefaultVal) #'\\tabnewline '
        sSize = LatexString2RST(sSize)
        if sSize.strip() != '':
            s += ', size = '+sSize
        # if sSymbol.strip() != '':
        #     s += ', symbol = '+RSTinlineMath(sSymbol.strip('$'))
        if sDefaultVal.strip() != '':
            s += ', default = ' + sDefaultVal
        s += ']:\n'
            
        s += RemoveIndentation(LatexString2RST(description), '  | ') + '\n'
        self.sRST += s

    #one row for table3, e.g. for output variables or any other \startTable from latex
    def Table3WriteRow(self, cols=['','',''], typeList=['','',''], nameLiteral=True):
        self.sLatex += cols[0] + ' & ' + cols[1] + ' & ' + cols[2] + '\\\\ \\hline\n'
        
        #RST:
        name = '**' + cols[0]+ '**'
        if nameLiteral:
            name = '``' + cols[0]+ '``'
        s = '* | '+name+'\\ : '
        
        if typeList[1] != '':
            s+=typeList[1]+', '
        s+=LatexString2RST(cols[1]) + '\n'
        
        #description:
        s += RemoveIndentation(LatexString2RST(cols[2]), '  | ') + '\n'
        self.sRST += s


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

# #************************************************
# #helper functions to create manual pybinding to access functions in classes
# #pyName = python name, cName=full path of function in C++, description= textual description used in C and in documentation
# #argList = [arg1Name, arg2Name, ...]
# #defaultArgs = [arg1Default, ...]: either empty or must have same length as argList
# #options= additional manual options (e.g. memory model)
# #example = string, which is put into latex documentation
# #isLambdaFunction = True: cName is intepreted as lambda function and copied into pybind definition
# def DefPyFunctionAccess(cClass, pyName, cName, description, argList=[], defaultArgs=[], example='', options='', isLambdaFunction = False): 
    
#     def ReplaceDefaultArgsCpp(s):
#         sNew = copy.copy(s)
#         sNew = sNew.replace('exu.','') #remove exudyn 'exu.' for C-code
#         sNew = sNew.replace('True','true').replace('False','false') #docu shows True, C++ code needs true
#         return sNew
    
#     def ReplaceDefaultArgsLatex(s):
#         sNew = copy.copy(s)
#         sNew = sNew.replace('true','True').replace('false','False')
#         if sNew.find('Vector3D') != -1:
#             sNew = sNew.replace('(std::vector<Real>)Vector3D','')
#             sNew = sNew.replace('{','').replace('}','')
#             sNew = sNew.replace('(','[').replace(')',']')
#         sNew = sNew.replace('py::','').replace('::','.') #replace C-style '::' (e.g. in ConfiguationType) to python-style '.'            
#         return sNew
#     #make some checks:
#     if (len(argList) != 0) & (len(defaultArgs) == 0):
#         defaultArgs = ['']*len(argList)
#     elif len(argList) != len(defaultArgs):
#         print('error in command '+pyName+': defaultArgs are inconsistent')
#         return ''
    
#     s = ''
#     if (cClass != ''):
#         s += '        .def("'
#     else:
#         s += '        m.def("'

#     #convert some special functions, like __repr__()
#     addBraces = True
#     pyNameLatex = pyName
#     if pyNameLatex in pyFunctionAccessConvert:
#         pyNameLatex = pyFunctionAccessConvert[pyName]
#         addBraces = False
#         #print('now pyName=', pyName)

#     s += pyName + '", ' 
#     if not(isLambdaFunction): #if lambda function ==> just copy cName as code
#         s += '&' 
#         if (cClass != ''):
#             s += cClass + '::'

#     s += cName + ', '
#     s += '"' + description +'"'
#     if (options != ''):
#         s += ', ' + options
    
    
#     sLatex = '  ' + Str2Latex(pyNameLatex)
#     if addBraces: sLatex += '('
#     if len(argList):
#         for i in range(len(argList)):
#             s += ', py::arg("' + argList[i] + '")'
#             sLatex += argList[i]
#             if (defaultArgs[i] != ''):
#                 sLatex += ' = ' + ReplaceDefaultArgsLatex(defaultArgs[i])
#                 s += ' = ' + ReplaceDefaultArgsCpp(defaultArgs[i])
#             sLatex += ', '
#         sLatex = sLatex[:-2] #remove last ', '

#     if addBraces: sLatex += ')'

#     s += ')'
            
#     if (cClass == ''):
#         s += ';'
    
#     s += '\n'


# #    sLatex += ' & '
# #    if len(defaultArgs):
# #        sLatex += '('
# #        for argStr in defaultArgs:
# #            sLatex += argStr + ' ,'
# #        sLatex = sLatex[:-2]+')'
    
#     sLatex += ' & ' + description.replace('_','\_')
#     if example != '':
#         example = Str2Latex(example)
#         example = example.replace('\\\\','\\tabnewline\n    ')
#         example = example.replace('\\TAB','\\phantom{XXXX}') #phantom spaces, not visible
#         sLatex += '\\tabnewline \n    \\textcolor{steelblue}{{\\bf EXAMPLE}: \\tabnewline \n    \\texttt{' + example.replace("'","{\\textquotesingle}") + '}}'
#     sLatex += '\\\\ \\hline \n'
    
#     return [s,sLatex]

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the following functions are used to generate pybinds to classes and add according latex documentation

# #start a new table to describe class bindings in latex;
# def DefLatexStartClass(sectionName, description, subSection=False, labelName=''):

#     sRST = ''
#     sLatex =  "\n%++++++++++++++++++++\n"
#     if subSection:
#         sLatex += "\\mysubsubsection"
#     else:
#         sLatex += "\\mysubsection"

#     sLatex += "{" + sectionName + "}\n"
#     if labelName != '':
#         sLatex += '\\label{sec:' +labelName+ '}\n'
#         sRST += RSTlabelString('sec-'+labelName) + '\n'
        
#     sLatex += description + '\n\n'
    
#     sLatex += '\\begin{center}\n'
#     sLatex += '\\footnotesize\n'
#     sLatex += '\\begin{longtable}{| p{8cm} | p{8cm} |} \n'
#     sLatex += '\\hline\n'
#     sLatex += '{\\bf function/structure name} & {\\bf description}\\\\ \\hline\n'

#     sRST += RSTheaderString(sectionName, 2) + '\n'
#     sRST += RemoveIndentation(description)

#     return [sLatex, sRST]

# def DefPyStartClass(cClass, pyClass, description, subSection = False):
#     s = '\n'
#     sectionName = pyClass
#     if (cClass == ''): 
#         #print("ERROR::DefPyStartClass: cClass must be a string")
#         sectionName = '\\codeName' #for EXUDYN, work around
        
#     if (cClass != ''):
#         s += '    py::class_<' + cClass + '>(m, "' + pyClass + '")\n'
#         s += '        .def(py::init<>())\n'

#     [sLatex, sRST] = DefLatexStartClass(sectionName, description, subSection=subSection)
        
#     return [s, sLatex, sRST]

# #finish latex table for class bindings 
# def DefLatexFinishTable():
#     sLatex = '\\end{longtable}\n'
#     sLatex += '\\end{center}\n'
    
#     return sLatex

# def DefPyFinishClass(cClass):
#     s = ''
    
#     if (cClass != ''):
#         s += '        ; // end of ' + cClass + ' pybind definitions\n\n'

#     sLatex = DefLatexFinishTable()
#     sRST = '\n'

#     return [s,sLatex,sRST]


# #add a enum value and definition to pybind interface and to latex documentation
# def AddEnumValue(className, itemName, description):
#     s = '		.value("' + itemName + '", ' + className + '::' + itemName + ')    //' + description + '\n'
#     #s = '		.value("' + itemName + '", ' + className + '::' + itemName + ', "' + description + '")\n' #does not work in pybind11
#     sLatex = '  ' + Str2Latex(itemName) + ' & ' + Str2Latex(description) + '\\\\ \\hline \n'
        
#     return [s,sLatex]


# #remove indentation of text block (for rst files) and afterwards add specific indentation:
# def RemoveIndentation2(text, addSpaces = ''):
#     isStartOfLine = True
#     s = addSpaces #created string
#     for x in text:
#         if isStartOfLine and x != '\n':
#             if x == ' ' or x == '\t': 
#                 continue
#             else: 
#                 isStartOfLine = False
#         else:
#             if x == '\n': 
#                 isStartOfLine = True
#                 s += '\n'
#                 s += addSpaces
#                 continue
#         s += x
#     return s

#remove indentation of text block (for rst files) and afterwards add specific indentation:
def RemoveIndentation(text, addSpaces = '', removeAllSpaces = True, removeIndentation = True):
    lines=text.replace('\t','    ').split('\n')
    s = ''
    hasEndl = False
    if lines[-1] == '':
        hasEndl = True
        del lines[-1]
    
    if not removeAllSpaces:
        minIndent = 10000
        for line in lines:
            if line != '':
                nSpaces = len(line)-len(line.lstrip(' '))
                minIndent=min(minIndent, nSpaces)
        
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

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the following functions were originally in pythonAutoGenerateObjects.py
    
#get list of filenames in folder dirPath which contain keyword (to find examples with specific items)
def ExtractExamplesWithKeyword(keyword, dirPath):
    from os import listdir
    from os.path import isfile, join
    
    fileNames = [f for f in listdir(dirPath) if isfile(join(dirPath, f))]
    
    filesWithKeyword = []

    for fileName in fileNames:
        if fileName[-3:]=='.py':
            #print("extract example:",fileName)
            file = open(dirPath+'/'+fileName)
            text = file.read()
            if text.find(keyword) != -1:
                filesWithKeyword += [fileName]
            file.close()
    return filesWithKeyword

#generate latex string containing a list of file references (and hyperref links), 
#based on a search through Examples and TestModels
#if latex is false, formatting is clean to be used in RST
def GenerateLatexStrKeywordExamples(itemType, itemName, itemShortName, useLatex = True):
    useLatex2 = True
    s = ''
    sRST = '' #if latex = False

    sepItem1 = '\\item '*useLatex2 #for items, put example in separate line, for utility functions, use one liner
    sepItem2 = '\n' 
    maxExamples = 12   #only 8 examples for utility functions
    initString = ''    #smaller font for utility function
    ufMode = False
    testModelString = ' (TestModels/)'
    examplesString = ' (Examples/)'

    if itemType != 'UtilityFunction':
        keywords = ['mbs.Add'+itemType+'('+itemName+'(']
        if itemName == 'ObjectRigidBody' or itemName == 'NodeRigidBodyEP':
            keywords += ['AddRigidBody('] #additional keyword
    
        if itemName == 'ObjectFFRF':
            keywords += ['AddObjectFFRF('] #additional keyword
    
        if itemName == 'ObjectFFRFreducedOrder':
            keywords += ['AddObjectFFRFreducedOrderWithUserFunctions('] #additional keyword

        if itemName == 'ObjectJointRevoluteZ':
            keywords += ['AddRevoluteJoint('] #additional keyword
        if itemName == 'ObjectPrismaticJointX':
            keywords += ['AddPrismaticJoint('] #additional keyword

    else:
        testModelString = ' (TM)'
        examplesString = ' (Ex)'
        ufMode = True
        keywords = [itemName + '(']
        sepItem1 = ', \n'     #for items, put example in separate line, for utility functions, use one liner
        sepItem2 = ''   #for items, put example in separate line, for utility functions, use one liner
        maxExamples = 5   #only 5+3 examples for utility functions
        initString = ' \\item \\footnotesize '*useLatex2    #smaller font for utility function


    if itemShortName != '' and itemName != itemShortName:
        keywords += ['mbs.Add'+itemType+'('+itemShortName+'(']

    processFolders = ['Examples','TestModels']
    folderAbrv = [examplesString, testModelString]
    cnt = 0 #examples counter
    sep = ''
    sepRST = ''
    headerCreated = False
    for iFolder, folder in enumerate(processFolders):
    
        fileListOrig = []
        for kw in keywords:
            dirPath = '../../pythonDev/'+folder
            fileListOrig += ExtractExamplesWithKeyword(keyword = kw,
                                                  dirPath = dirPath)
        
        fileList = []
        for f in fileListOrig:
            if f not in fileList: fileList += [f]
    
        if len(fileList) != 0:
            if not headerCreated:
                headerCreated = True
                sComment = '%\n\\noindent For examples on '+itemName+' see '
                s += sComment*useLatex2
                
                if ufMode:
                    sExTest = 'Relevant Examples (Ex) and TestModels (TM) with weblink to github:\n'
                else:
                    sExTest = 'Relevant Examples and TestModels with weblink:\n'
                s += sExTest
                sRST += sExTest

                s += '\\bi\n'
                s += initString
                sep = ''
                sRST += '\n    ' #in RST, we could put everything into one list ...
            
            if not ufMode:
                sep = sepItem1
            for name in fileList:
                fileURL = 'https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/'+folder+'/' + name
                s += sep+'\\exuUrl{'+fileURL+'}'
                s += '{\\texttt{'+name.replace('_','\\_')+'}}' 
                sRST += sepRST
                sepRST = ', '
                sRST += RSTurl(name, fileURL,False, True)+folderAbrv[iFolder]
                
                s += folderAbrv[iFolder]
                s += sepItem2
                sep = sepItem1
    
                cnt += 1
                if (cnt % 3 == 0) and ufMode: sep = sep+'\\\\ '*useLatex2 #shorten lines a little
                if cnt >= maxExamples+3*iFolder: #some functions would appear in all examples... 
                    s += sepItem1 + ' ...' + sepItem2 + '\n'
                    break

    if headerCreated:
        s += '\n\\ei\n'
        s += '\n%\n'
        sRST += '\n\n'

    if useLatex:
        return s
    else:
        return [s, sRST]







