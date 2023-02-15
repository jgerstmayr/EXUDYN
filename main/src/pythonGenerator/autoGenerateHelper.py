# -*- coding: utf-8 -*-
"""
Created on Fri May 18 08:53:30 2018

@author: Johannes Gerstmayr

goal: automatically generate interfaces for structures
currently: automatic generate structures with ostream and initialization
"""
import datetime # for current date
import copy

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


#************************************************
#convert string to doxygen readable comment --> for formulas in comments and class descriptions
def Str2Doxygen(s, isDefaultValue=False): #replace _ and other symbols to fit into latex code

    s = s.replace('$','\\f$') #$ must be written as \f$ in doxygen
    s = s.replace('\\be','\\f[') #$ must be written as \f$ in doxygen
    s = s.replace('\\ee','\\f]') #$ must be written as \f$ in doxygen
    s = s.replace('\\bi','') #not needed in doxygen
    s = s.replace('\\ei','') #not needed in doxygen
    s = s.replace('\item[]','') #not needed in doxygen
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
    s = s.replace('_','\_')
    if replaceCurlyBracket: #don't do that for systemstructures definitions, allowing hyperlinks, etc.
        s = s.replace('{','\{')
        s = s.replace('}','\}')
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
            returnStr += commaStr+'\\texttt{'+t.replace('_','\_')+'}'
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
    s = s.replace('PyMatrixContainer()','[]')  #initialization in iteminterface with empty array
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
        s = s.replace('Vector6DList','') #KinematicTree
        s = s.replace('Vector3DList','') #KinematicTree
        s = s.replace('Matrix3DList','') #KinematicTree
        s = s.replace('Vector2DList','') #BeamSectionGeometry
        s = s.replace('PyVector2DList','') #BeamSectionGeometry
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
    #s = s.replace('_','\_') 
    #s = s.replace('{','\{')
    #s = s.replace('}','\}')
    #s = s.replace('$','\$')

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
    '\\ra':'\\rightarrow',
    '\\LU':'^',
    }

convLatexWords={'(\\the\\month-\\the\\year)':'',
           '    \item':'\item',
           '  \item':'\item',
           '\item[$\\ra$]':'  |  → ', #'+ ->', #this is always a sublist
           '\\item[]':'  ', 
           '\\item[--]':' - ',  #one additional whitespace at beginning for alignment of sub-lists!
           '\\item':'+ ',
           '\\small':'',
           '\\noindent ':'',
           '\\noindent':'',
           '\\phantom{XXXX}':'    ',
           '$\\ra$':'→',
           '\\newpage':'',
           '\\horizontalRuler':'',
           '\\\\':'\n\n',
           '$\\backslash$':'\\',
           '\\plainlststyle':'',
           '\\codeName\\':'Exudyn',
           '\\codeName':'Exudyn',
           '\\pythonstyle\\begin{lstlisting}':'\n.. code-block:: python\n',
           '\\begin{lstlisting}':'\n.. code-block::\n',
           '\\end{lstlisting}':'\n',
           '\\begin{center}':'',
           '\\end{center}':'',
           '\\includegraphics[height=6cm]{../demo/screenshots/plotSpringDamper}':'see theDoc.pdf',
           # '+++++++++++++++++++++++++++++++':'\\ +++++++++++++++++++++++++++++++\n', #special problems with .rst
           # '=========================================':'\\ =========================================\n', #special problems with .rst
           '\\begin{itemize}':'', 
           '[leftmargin=1.4cm]':'',
           '[leftmargin=1.2cm]': '',
           '[leftmargin=0.5cm]':'', 
           '\\rule{8cm}{0.75pt}':'', 
           '\\textcolor{steelblue}':'', 
           '[language=Python, xleftmargin=36pt]':'',
           '': '',
           '': '',
           '': '',

           '\\bi':'', 
           '\\ei':'',
           '\\bn':'', 
           '\\en':'',
           '\\be':'', 
           '\\ee':'',
           '\\it ':'',
           #specials:
           '\\ge':'>=',
           '\\_':'_',
           '\\textdegree':'°',
           '\\ac':'',
           '\\acs':'',
           '\\acp':'',
           '\\acf':'',
           #
           '{\"a}':'ä',
           '{\"o}':'ö',
           '{\"u}':'ü',
           '\\"a':'ä', #if '{' is already removed earlier
           '\\"o':'ö',
           '\\"u':'ü',
           '{':'',
           '}':'',
           '$':'',
           }
    
convLatexCommands={#(precommand,'_USE'/'',postcommand)
    '\\ignoreRST':('','',''),
    '\\texttt':('\\ ``','_USE','``\\ '),
    '\\label':('\n\n.. _','_USE',':\n\n'), #do this before sections ...
    '\\mysectionlabel':('','',''),
    '\\mysubsectionlabel':('','',''),
    '\\mysubsubsectionlabel':('','',''),
    '\\mysubsubsubsectionlabel':('','',''),
    '\\mysection':('','',''),
    '\\mysubsection':('','',''),
    '\\mysubsubsection':('','',''),
    '\\mysubsubsubsection':('','',''),
    '\\pythonListing':('','',''),
    '\\pythonSmallListing':('','',''),
    '\\smallListing':('','',''),
    '\\myListing':('','',''),
    '\\setlength':('','',''),
    '\\vspace':('','',''),
    '\\footnote':(' (','_USE',')'),
    '\\mybold':('\\ **','_USE','**\\ '),
    '\\mathrm':('','_USE',''),
    '\\cite':('','',''),
    '\\onlyRST':('','_USE',''),
    #'\\refSection':('theDoc.pdf','',''),
    #'\\refSection':(' `','_USE','`_\\ '),
    '\\refSection':(' :ref:`','_USE','`\\ '),
    '\\fig':('[figure in theDoc.pdf]','',''),
    '\\exuUrl':('`','_USE','`_'),
    '\\ref':('[theDoc.pdf]','',''),
    'figure':('','',''),
    } #TITLE, SUBTITLE, SUBSUBTITLE, ...

#replace all occurances of conversionDict in string and return modified string
def ReplaceWords(s, conversionDict): #replace strings provided in conversion dict
    for (key,value) in conversionDict.items():
        s = s.replace(key, value)

    return s

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#add specific markup with blind spaces
def RSTmarkup(name, c='*', blindSpaces=True):
    return '\\ '*blindSpaces+c+name+c+'\\ '*blindSpaces


#add code block; code must already be indented; code must have \n at end
def RSTcodeBlock(code, typeString=''):
    s = '.. code-block:: '+typeString + '\n\n'
    s += code + '\n'
    return s

#create text for inline URL
def RSTurl(urlText, link, boldFace=False, blindSpaces=False):
    bf = ''
    if boldFace:
        bf = ':stlink:'
    return '\\ '*blindSpaces+bf+'`'+urlText+' <'+link+'>`_'+'\\ '*blindSpaces


#write latex source inline
def RSTinlineMath(mathString):
    s = '\\ :math:`' + mathString + '`\\ '
    return s

#write latex source in separate equation
def RSTmath(mathString, label=''):
    s = '.. math:: '+mathString+'\n'
    if label!='':
        s+='   :label: '+label + '\n'
    s += '\n'
    return s

#label string directly to be placed e.g. before header
def RSTlabelString(name):
    return '\n.. _'+name.replace(':','-').replace('_','-').lower()+':\n'


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
    return s

#start searching for { and matching } bracket, including sub-brackets
def FindMatchingBracket(s, start, openBracket='{', closingBracket='}'):
    cnt = 0
    bStart = -1
    for i in range(start,len(s)):
        if s[i] == openBracket:
            cnt += 1
            bStart = i
        elif s[i] == closingBracket:
            cnt -= 1

        if bStart != -1 and cnt == 0:
            return [bStart,i]
    return [-1,-1]
        
#convert a text that is mainly designed for latex, but to be output into RST
def LatexString2RST(s, replaceCommands=True, replaceMarkups = False): #Latex style to RST

    # replace latex math to RST inline
    sNew = ''
    endFound = False
    pos = 0

    while not endFound:
        val = s.find('$',pos)
        if val == -1:
            val = len(s)
            endFound = True
        regText = s[pos:val]
        if replaceMarkups:
            regText = regText.replace('*','\\*')
            
        # regText = regText.replace('\\{','{')
        # regText = regText.replace('\\}','}')
        # #regText = regText.replace('\\_','_') #no need to replace, as we also need \_ in RST !
        # regText = regText.replace('\&','&')
        if replaceCommands:
            regText = ReplaceLatexCommands(regText , convLatexCommands)
            regText = ReplaceWords(regText , convLatexWords)


        sNew += regText
        if not endFound:
            val += 1
            valEnd = s.find('$',val)
            if valEnd != -1:
                sMath = s[val:valEnd]
                sMath = ReplaceWords(sMath, convLatexMath)
                sNew += RSTinlineMath(sMath) #do not replace markups inside this text
                val = valEnd+1
            else:
                print('WARNING:\nutilities: found no closing $ in:\n', s)
                endFound = True
                sNew = s
        pos = val

    # sNew = sNew.replace('\\"u','ü')
    # sNew = sNew.replace('\\"a','ä')
    # sNew = sNew.replace('\\"o','ö')
    
    # sNew = sNew.replace('\\codeName\\ ','Exudyn')
    # sNew = sNew.replace('\\codeName','Exudyn')
    return sNew

    
    return s

#if key is found, return [preString, innerString, innerString2, postString], otherwise -1; '123\section{abc}456' = ['123','abc','456']
#if secondBracket=True, it searches two consecutive brackets: \exuURL{...}{...} and stores in innerString2
def ExtractLatexCommand(s, key, secondBracket, isBeginEnd):
    found = -1
    if isBeginEnd: #find \begin{...} \end{...}
        #always find next occurances --> will be erased in next run ...
        sStart = s.find('\\begin{'+key+'}')
        sEnd = s.find('\\end{'+key+'}')
        if sStart == -1 or sEnd == -1:
            return -1
        else:
            found = sStart
            sStart += len('\\begin{'+key+'}') - 1
            #sEnd += len('\\end{'+key+'}') - 1
            preString = s[:found]
            innerString = s[sStart+1:sEnd]
            postString = s[sEnd+len('\\end{'+key+'}'):]

            return [preString, innerString, '', postString]
    else:
        found = s.find(key)
        if found != -1:
            [sStart, sEnd] = FindMatchingBracket(s, found)

            preString = s[:found]
            if sEnd == -1:
                print('no matching bracket found: '+key+', "'+s[found:min(found+20,len(s))]+'"')
                raise ValueError('ERROR')
    
            innerString = s[sStart+1:sEnd]
            postString = s[sEnd+1:]
    
            sStart2 = -1
            sEnd2 = -1
            innerString2 = ''
            if secondBracket:
                [sStart2, sEnd2] = FindMatchingBracket(s, sEnd+1)
                if sEnd2 == -1:
                    # print("start ", sStart, ", end ", sEnd)
                    print('no matching second bracket found: '+key+', "'+s[sStart+1:sStart+50]+'"')
                    raise ValueError('ERROR')
                innerString2 = s[sStart2+1:sEnd2]
                postString = s[sEnd2+1:]
    
            # print(sStart, sEnd)
            return [preString, innerString, innerString2, postString]
        else:
            return -1

def ReplaceLatexCommands(s, conversionDict): #replace strings provided in conversion dict
    s = s.replace('{\\bf ','\\mybold{') #this is then further converted into rst code ...
    for (key,value) in conversionDict.items():
        found = 0
        while (found != -1):
            secondBracket = False
            isBeginEnd = False
            if key == '\\exuUrl' or 'sectionlabel' in key: 
                secondBracket = True
            if key == 'figure': isBeginEnd = True

            found = ExtractLatexCommand(s, key, secondBracket, isBeginEnd)
            if found != -1:
                [preString, innerString, innerString2, postString] = found
                # if isBeginEnd:
                #     print("inner="+innerString+'+++')
                # if key == '\\label':
                #     #label in rst needs to be put in front of section ... put in front of 
                #     nLast = max(0,preString.rfind('\n',0,-1)) #start one char earlier to catch direct \n before; otherwise take start of string
                #     s = preString[0:nLast]
                #     s += value[0] + innerString.replace(':','-').lower() + value[2]
                #     s += preString[nLast:] #this may be the header
                # else:
                s = preString
                if key == '\\refSection' or key == '\\label':
                    innerString=innerString.replace(':','-').lower()
                else:
                    innerString = ReplaceWords(innerString, convLatexWords) #needs to be cleaned here already
                
                if key == '\\mysection' or key == '\\mysectionlabel':
                    if 'label' in key: s += RSTlabelString(innerString2)+'\n'
                    s += '\n'+RSTheaderString(innerString, 1)
                elif key == '\\mysubsection' or key == '\\mysubsectionlabel':
                    if 'label' in key: s += RSTlabelString(innerString2)+'\n'
                    s += '\n'+RSTheaderString(innerString, 2)
                elif key == '\\mysubsubsection' or key == '\\mysubsubsectionlabel':
                    if 'label' in key: s += RSTlabelString(innerString2)+'\n'
                    s += '\n'+RSTheaderString(innerString, 3)
                elif key == '\\mysubsubsubsection' or key == '\\mysubsubsubsectionlabel':
                    if 'label' in key: s += RSTlabelString(innerString2)+'\n'
                    s += '\n'+RSTheaderString(innerString, 4)
                elif key == '\\exuUrl':
                    s += value[0]
                    s += innerString2 + ' <' + innerString + '>'
                    s += value[2]
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
    def __init__(self, sPy='', sLatex='', sRST=''):
        
        self.sPy = sPy
        self.sLatex = sLatex
        self.sRST = sRST
        self.rstFileLists = [] #contains tuples (filename, text)
        self.rstCurrentFileName = '' #if this is non-empty, it will be stored in list with current text
        
    #**classFunction: add (+) operator allows adding another inertia information with SAME local coordinate system and reference point!
    #only inertias with same center of rotation can be added!
    #**example: 
    #J = InertiaSphere(2,0.1) + InertiaRodX(1,2)
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
        #self.sRST += ' `'+ref.replace(':','-').lower()+'`_\\ '
        self.sRST += ' :ref:`'+ref.replace(':','-').lower()+'`\\ '

        
    #add python style code blocks to latex and RST
    def AddDocuCodeBlock(self, code, pythonStyle=True):
        # print('code0=', ord(code[0]))
        if code.strip(' ')[-1] != '\n':
            code += '\n'
        self.sLatex +='\\pythonstyle\n'
        self.sLatex +='\\begin{lstlisting}[language=Python, firstnumber=1]\n'
        self.sLatex += code
        self.sLatex +='\\end{lstlisting}\n\n'

        self.sRST += '\n.. code-block:: ' + 'python'*pythonStyle + '\n' #needs empty line in between
        self.sRST += '\n'*(code.strip(' ')[0] != '\n')
        self.sRST += RemoveIndentation(code, '   ',False)
        #print(RemoveIndentation(code, '   '))
        self.sRST += '\n'*(code.strip(' ')[-1] != '\n')
        

    #add python style code blocks to latex and RST
    def AddDocuList(self, itemList, itemText=''):
        if len(itemList) != 0:
            self.sLatex += '\\bi\n'
            self.sRST += '\n'
            for item in itemList:
                sEnd = '\n'*(item.strip(' ')[-1] != '\n') #add separator if not there already
                self.sLatex +='\item'+itemText+' ' + item + sEnd
                if itemText == '':
                    rstItem = '*'
                elif itemText == '[]':
                    rstItem = ' '
                else: 
                    print('WARNING: AddDocuList: illegal itemText:'+itemText)
                
                self.sRST += rstItem + RemoveIndentation(LatexString2RST(item), '  | ')[1:] + sEnd
    
            self.sRST += '\n'
            self.sLatex += '\\ei'

    #start a new table to describe class bindings in latex;
    def DefLatexStartTable(self, classStr=''):
        self.sLatex += '\\begin{center}\n'
        self.sLatex += '\\footnotesize\n'
        self.sLatex += '\\begin{longtable}{| p{8cm} | p{8cm} |} \n'
        self.sLatex += '\\hline\n'
        self.sLatex += '{\\bf function/structure name} & {\\bf description}\\\\ \\hline\n'

        #self.sRST += '\n\ **Description of functions and structures**:\n\n'
        addInfo = ''
        if ':' in classStr:
            ni = classStr.find(':')
            addInfo = ' regarding **'+classStr[ni+1:]+'**'
            classStr = classStr[:ni]
        self.sRST += '\n\ The class **'+classStr+'** has the follwing **functions and structures**'+addInfo+':\n\n'

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
    def DefPyStartClass(self, cClass, pyClass, description, subSection = False, labelName=''):
        self.sPy += '\n'
        sectionName = pyClass
        if (cClass == ''): 
            #print("ERROR::DefPyStartClass: cClass must be a string")
            sectionName = '\\codeName' #for EXUDYN, work around
            
        if (cClass != ''):
            self.sPy += '    py::class_<' + cClass + '>(m, "' + pyClass + '")\n'
            self.sPy += '        .def(py::init<>())\n'
    
        self.DefLatexStartClass(sectionName, description, subSection=subSection, labelName=labelName)
    
    #finish latex table for class bindings 
    def DefLatexFinishClass(self):
        self.sLatex += '\\end{longtable}\n'
        self.sLatex += '\\end{center}\n'
        self.sRST += '\n\n' #empty line closes list block

    def DefPyFinishClass(self, cClass):
        
        if (cClass != ''):
            self.sPy += '        ; // end of ' + cClass + ' pybind definitions\n\n'
    
        self.DefLatexFinishClass()
        self.sRST += '\n'

    #add latex table entry / RST list entry for data variable
    def DefLatexDataAccess(self, name, description): 
        self.sLatex += '  ' + Str2Latex(name) + ' & '+Str2Latex(description) + '\\\\ \\hline  \n'
        self.sRST += '* | ' + '**'+Str2Latex(name)+'**:\n'
        self.sRST += RemoveIndentation(LatexString2RST(description), '  | ') + '\n'
        
    #************************************************
    #helper functions to create manual pybinding to access functions in classes
    #pyName = python name, cName=full path of function in C++, description= textual description used in C and in documentation
    #argList = [arg1Name, arg2Name, ...]
    #defaultArgs = [arg1Default, ...]: either empty or must have same length as argList
    #options= additional manual options (e.g. memory model)
    #example = string, which is put into latex documentation
    #isLambdaFunction = True: cName is intepreted as lambda function and copied into pybind definition
    def DefPyFunctionAccess(self, cClass, pyName, cName, description, argList=[], defaultArgs=[], example='', options='', isLambdaFunction = False): 
        
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
            for i in range(len(argList)):
                self.sPy += ', py::arg("' + argList[i] + '")'
                sLadd += argList[i]
                sRadd += '\\ *'+argList[i]+'*\\ '
                if (defaultArgs[i] != ''):
                    self.sPy += ' = ' + ReplaceDefaultArgsCpp(defaultArgs[i])
                    sLadd += ' = ' + ReplaceDefaultArgsLatex(defaultArgs[i])
                    sRadd += ' = ' + ReplaceDefaultArgsLatex(defaultArgs[i])
                sLadd += ', '
                sRadd += ', '
            sLadd = sLadd[:-2] #remove last ', '
            sRadd = sRadd[:-2] #remove last ', '
        self.sLatex += sLadd
        self.sRST += sRadd
        
        if addBraces: 
            self.sLatex += ')'
            self.sRST += ')'
    
        self.sPy += ')'
                
        if (cClass == ''):
            self.sPy += ';'
        
        self.sPy += '\n'

        self.sLatex += ' & ' + description.replace('_','\_')
        #self.sRST += ': \n' +  RemoveIndentation(description.replace('_','\_'), '  | ') + '\n'
        self.sRST += ': \n' +  RemoveIndentation(LatexString2RST(description), '  | ') + '\n'
        if example != '':
            example = Str2Latex(example)
            exampleRST = example
            example = example.replace('\\\\','\\tabnewline\n    ')
            example = example.replace('\\TAB','\\phantom{XXXX}') #phantom spaces, not visible
            self.sLatex += '\\tabnewline \n    \\textcolor{steelblue}{{\\bf EXAMPLE}: \\tabnewline \n    \\texttt{' + example.replace("'","{\\textquotesingle}") + '}}'
            exampleRST = exampleRST.replace('\\\\','\n')
            
            self.sRST += '  | *Example*:\n\n'
            self.sRST += '  '+RSTcodeBlock(RemoveIndentation(exampleRST,'   '+'  ', False).replace('\\TAB','    '), 'python') + '\n'
        self.sLatex += '\\\\ \\hline \n'

    #add a enum value and definition to pybind interface and to latex documentation
    def AddEnumValue(self, className, itemName, description):
        self.sPy += '		.value("' + itemName + '", ' + className + '::' + itemName + ')    //' + description + '\n'

        #self.sLatex += '  ' + Str2Latex(itemName) + ' & ' + Str2Latex(description) + '\\\\ \\hline \n'
        self.DefLatexDataAccess(itemName, description) #Str2Latex(...) done inside function



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
# def DefLatexFinishClass():
#     sLatex = '\\end{longtable}\n'
#     sLatex += '\\end{center}\n'
    
#     return sLatex

# def DefPyFinishClass(cClass):
#     s = ''
    
#     if (cClass != ''):
#         s += '        ; // end of ' + cClass + ' pybind definitions\n\n'

#     sLatex = DefLatexFinishClass()
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
                    sExTest = 'Relevant Examples (Ex) and TestModels (TM) with weblink:\n'
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
                s += sep+'\exuUrl{'+fileURL+'}'
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







