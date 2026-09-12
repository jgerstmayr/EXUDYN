#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an Exudyn file
#
# Details:  this file is used to create generic Exudyn examples
#
# Author:   Johannes Gerstmayr
# Date:     2023-08-22
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import io   #for utf-8 encoding
import sys
import numpy as np
# from os import listdir
# from os.path import isfile, join
from copy import copy

from numpy.random import rand, randint
import exudyn as exu
# import inspect
# import re 

np.random.seed(42)

commandDelimiters = " \\$+-/*=~&!?.,;:{}[]'()_^\n\t#|<>"

listUnknownCommands = []

#find searchString, but ignore comments!
def SafeFind(text, searchString, start, end=None, optional=True, isCommand=True):
    if end == None: end=len(text)
    
    pos = text.find(searchString, start, end)
    if pos == -1: 
        if optional: 
            return pos
        else:
            raise ValueError('did not find: "'+searchString+'" in: '+text[start-20:start+20])

    if pos+len(searchString)+1 < len(text):
        if text[pos+len(searchString)-1] not in commandDelimiters and text[pos+len(searchString)] not in commandDelimiters:
            return SafeFind(text, searchString, pos+len(searchString), end, optional, isCommand)
    
    posComment = text.rfind('%',start, pos) #find comment closest to pos
    if posComment != -1:
        if text.find('\n',posComment,pos) == -1: #check if there is a space between comment and pos
            #print('found "'+searchString+'" in comment line:'+ text[posComment:pos+20])
            return SafeFind(text, searchString, pos+len(searchString), end, optional, isCommand)

    return pos


#find closing text, where current position is after current openText
def FindMatchingClosingText(s, start, openText='\\bi', closingText='\\ei', isOpen=True, isCommand=True):
    cnt = int(isOpen)
    i = start
    if not isOpen:
        i = SafeFind(s, openText, start, isCommand=True)
        if i==-1: ValueError('FindMatchingClosingText: no openText found: '+ s[start-20:start+20])

    while True:
        nextOpen = SafeFind(s, openText, i, isCommand=True)
        nextClose = SafeFind(s, closingText, i, isCommand=True)
        debug = False
        if nextClose == -1:
            raise ValueError('FindMatchingClosingText: no closing found: '+ s[start-20:start+20])
        if nextOpen == -1: 
            if cnt > 1:
                if debug: print('close1:cnt -= 1')
                cnt -= 1
                i = nextClose + len(closingText)
            else:
                if debug: print('nextClose1')
                return nextClose
        else:
            if nextOpen < nextClose:
                i = nextOpen + len(openText)
                cnt += 1
                if debug: print('nextOpen < nextClose')
            elif cnt == 1:
                if debug: print('nextClose2')
                return nextClose
            else: 
                if debug: print('close2:cnt -= 1')
                cnt -= 1
                i = nextClose + len(closingText)
                

#start searching for { and matching } bracket, including sub-brackets
def FindMatchingBracket(s, start, openBracket='{', closingBracket='}'):
    cnt = 0
    bStart = -1
    if start == -1: raise ValueError('FindMatchingBracket: search position is -1')
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


#words which are always replaced in latex texts (in all modes, equations, codes, ...)
directConversion = {
    '{\\bf':'\\mybold{',
    '{\\it':'\\myitalics', 
    '{\\small':'\\mysmall{',
    '(\\the\\month-\\the\\year)':'',
    '[leftmargin=1.4cm]':'',
    '[leftmargin=1.2cm]': '',
    '[leftmargin=0.5cm]':'', 
    '$\\ra$':'→',
    '-{}-':'--',
    '{\\"a}':'ä',
    '{\\"o}':'ö',
    '{\\"u}':'ü',
    '\\"a':'ä', 
    '\\"o':'ö',
    '\\"u':'ü',
    '$\\backslash$':'\\backslash{}',
    '[language=Python, xleftmargin=36pt]':'',
    '\\phantom{XXXX}':'    ',
    '\\rule{8cm}{0.75pt}':'', 
    '\\textcolor{steelblue}':'', 
    '\\pythonstyle\\begin{lstlisting}':'\\begin{pytlisting}',
    '    \\item':'\\item',
    '  \\item':'\\item',
    '\\begin{itemize}':'\\bi', 
    '\\end{itemize}':'\\ei', 
    '\\begin{itemize}':'\\bi', 
    '\\end{itemize}':'\\ei', 
    }

#commands with no args; replacements for RST
noArgCommands = {
   # '\\item[$\\ra$]':'  |  → ', #'+ ->', #probably not used any more
   # '\\item[]':'  ', 
   # '\\item[--]':' - ',  #one additional whitespace at beginning for alignment of sub-lists!
   # '\\item':'+ ',
   #'\\codeName\\':'Exudyn',
   # '\\begin{center}':'',
   # '\\end{center}':'',
   # '\\begin{itemize}':'', 
   # '\\bi':'', 
   # '\\ei':'',
   # '\\bn':'', 
   # '\\en':'',
   '\\finishTable':'',
   '\\noindent':'',
   '\\nonumber':'', 
   '\\textbar':'|',
   '\\lbrack':'[',
   '\\rbrack':']',
   '\\newpage':'\n',
   '\\tabnewline':'',
   '\\horizontalRuler':'',
   '\\plainlststyle':'',
   '\\codeName':'Exudyn',
   '\\pythonstyle':'',
   '\\ge':'>=',
   '\\{':'{',
   '\\}':'}',
   '\\[':'[',
   '\\]':']',
   '\\_':'_',
   '\\ ':' ',
   '\\&':'&',
   '\\#':'#',
   '\\%':'%',
   '\\textdegree':'°',
   '\\rstStartNewLine':'\\ ',
   '\\\\':'',
   '\\eqDot': '.',
   '\\eqComma': ',',
   '\\ImThree': '\\mathbf{I}_{3 \\times 3}',
   '\\ImTwo': '\\mathbf{I}_{2 \\times 2}',
   '\\textdegree':'°',

   }

#these commands are kept as they are (otherwise we could ignore unknown commands)
mathCommands = [
    '\\.','\\,','\\!','\\;', '\\in','\\mu','\\times',
 '\\ddot',
 '\\frac',
 '\\partial',
 '\\tlambda',
 '\\SO',
 '\\dot',
 '\\FO',
 '\\txi',
 '\\tp',
 '\\vel',
 '\\Delta',
 '\\int',
 '\\tau',
 '\\left',
 '\\ldots',
 '\\right',
 '\\approx',
 '\\rm',
 '\\vdots',
 '\\hat',
 '\\xi',
 '\\cdot',
 '\\le',
 '\\sqrt',
 '\\sum',
 '\\mathrm',
 '\\quad',
 '\\beta',
 '\\gamma',
 '\\alpha',
 '\\SON',
 '\\FON',
 '\\aalg',
 '\\AEN',
 '\\rho',
 '\\omega',
 '\\bar',
 '\\neq',
 '\\infty',
 '\\GA',
 '\\acc',
 '\\vr',
 '\\mr',
 '\\AE',
 '\\Null',
 '\\mbox',
 '\\Rcal',
 '\\min',
 '\\epsilon',
 '\\par',
 '\\rule',
 '\\cRef',
 '\\cIni',
 '\\LU',
 '\\psi',
 '\\tpsi',
 '\\cConfig',
 '\\theta',
 '\\Rot',
 '\\pLocB',
 '\\pLoc',
 '\\ttheta',
 '\\tomega',
 '\\pi',
 '\\nu',
 '\\tnu',
 '\\varphi',
 '\\sin',
 '\\cos',
 '\\cCur',
 '\\prime',
 '\\pRefG',
 '\\returnValue',
 '\\pRef',
 '\\mp',
 '\\vp',
 '\\ttau',
 '\\talpha',
 '\\lambda',
 '\\tilde',
 '\\mathbf',
 '\\tphi',
 '\\tPhi',
 '\\mathrel',
 '\\indf',
 '\\indr',
 '\\indt',
 '\\indrigid',
 '\\delta',
 '\\indtt',
 '\\indtf',
 '\\indrr',
 '\\indtr',
 '\\indrf',
 '\\omegaBDtilde',
 '\\Phi',
 '\\indred',
 '\\induser',
 '\\tPsi',
 '\\tzeta',
 '\\tsigma',
 '\\teps',
 '\\ll',
 '\\zeta',
 '\\text',
 '\\tTheta',
 '\\LUX',
 '\\tchi',
 '\\indu',
 '\\indff',
 '\\Psi',
 '\\otimes',
 '\\widetilde',
 '\\ddots',
 '\\varepsilon',
 '\\kappa',
 '\\Vert',
 '\\qquad',
 '\\bullet',
 '\\pm',
 '\\cdots',
 '\\LUR',
 '\\LURU',
 '\\diag',
 '\\displaystyle',
 '\\tmu',
 '\\phi',
 '\\end',
 '\\ee',
 '\\lVert',
 '\\rVert',
 '\\pluseq',
 '\\Omega',
 '\\Pi',
 '\\cVis',
 '\\cSOS',
 '\\vrRow',
 '\\vec',
 '\\label',
 '\\eta',
 '\\underbrace',
 '\\SYS',
 '\\SYSN',
 '\\implies',
 '\\dd',
 '\\mathcal',
 '\\det',
 '\\tvarphi',
 '\\ra',
 '\\vert',
 '\\co',
 '\\si',
 '\\tbeta',
 '\\vfour',
 '\\equiv',
 '\\ps',
 '\\myoverline',
 '\\circ',
 '\\LLdot',
 '\\clearpage',
 '\\termC',
 '\\termA',
 '\\diffmOI',
 '\\diffmOIt',
 '\\diffmOt',
 '\\diffmIt',
 '\\tdelta',
 '\\ANCFdk',
 '\\diffmI',
 '\\diffANCF',
 '\\diffANCFt',
 '\\ANCFdkt',
 '\\diffANCFmI',
 '\\ANCFdkOtp',
 '\\diffANCFdk',
 '\\ANCFdkO',
 '\\diffANCFmIt',
 ]

keepCommands = ["\\'"] + mathCommands 

abc = 'abcdefghijklmnopqrstuvwxyz'
for c in abc:
    noArgCommands['\\'+c+'v'] = '{\\mathbf{'+c+'}}'
    noArgCommands['\\'+c.upper()+'m'] = '{\\mathbf{'+c.upper()+'}}'


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#special functions called by creators
def CreateSection(command, arg0, arg1=''):
    d = {'type':'SECTION', 'LEVEL': command.count('sub'), 'TEXT':arg0}
    if arg1:
        d['LABEL'] = arg1
    return [d]

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#commands with several args including conversion rule to RST
multiArgCommands = {
    '\\backslash':{'nArgs':1, 'PRE':'\\'}, #workaround for \\
    '\\ignoreRST':{'nArgs':1, 'action':'REMOVE'},
    '\\onlyRST':{'nArgs':1, 'action':'RST'},
    '\\mysection':{'nArgs':1, 'CALL': CreateSection},
    '\\mysectionlabel':{'nArgs':2, 'CALL': CreateSection},
    '\\mysubsectionlabel':{'nArgs':2, 'CALL': CreateSection},
    '\\mysubsubsectionlabel':{'nArgs':2, 'CALL': CreateSection},
    '\\mysubsubsubsectionlabel':{'nArgs':2, 'CALL': CreateSection},
    '\\mysubsection':{'nArgs':1, 'CALL': CreateSection},
    '\\mysubsubsection':{'nArgs':1, 'CALL': CreateSection},
    '\\mysubsubsubsection':{'nArgs':1, 'CALL': CreateSection},
    # '\\ignoreRST':{'nArgs':1, 'USE':False},

    '\\paragraph':{'nArgs':1, 'PRE':'\n\\ **','POST':'** ', 'type':'PARAGRAPH'},
    '\\setlength':{'nArgs':1, 'action':'REMOVE'},
    '\\vspace':{'nArgs':1, 'action':'REMOVE'},

    '\\footnote':{'nArgs':1, 'PRE':'\\ (','POST':')', 'type':'FORMAT'},
    '\\texttt':{'nArgs':1, 'PRE':'\\ ``','POST':'``\\ ', 'type':'FORMAT'},
    '\\mybold':{'nArgs':1, 'PRE':'\\ **','POST':'**\\ ', 'type':'FORMAT'},
    '\\myitalics':{'nArgs':1, 'PRE':'\\ *','POST':'*\\ ', 'type':'FORMAT'},
    '\\mysmall':{'nArgs':1, 'type':'FORMAT'},
    '\\underline':{'nArgs':1, 'type':'FORMAT'},

    '\\cite':{'nArgs':1, 'action':'REMOVE'},

    '\\setlength':{'nArgs':1, 'action':'REMOVE'},
    '\\setlength':{'nArgs':1, 'action':'REMOVE'},

    '\\userFunctionExample':{'nArgs':1, 'PRE':'\n--------\n\n\\ **User function example**\\ :\n\n', 'type':'PARAGRAPH'},
    '\\userFunction':{'nArgs':1, 'PRE':'\n--------\n\n\\ **Userfunction**\\ : ``','POST':'`` \n\n', 'type':'PARAGRAPH'},

    '\\LatexRSTfigure':{'nArgs':5, 'type':'FIGURE'},

    #for now, tables are not treated as structures:
    '\\startGenericTable':{'nArgs':1, 'PRE':'\n.. list-table:: \\ \n   :widths: auto\n   :header-rows: 1\n', 'type':'TABLE'},
    '\\rowTable'     :{'nArgs':3, 'PATTERN':'   * - | _ARG0_\n'+'     - | _ARG1_\n'+'     - | _ARG2_\n'},
    '\\rowTableThree':{'nArgs':3, 'PATTERN':'   * - | _ARG0_\n'+'     - | _ARG1_\n'+'     - | _ARG2_\n'},
    '\\rowTableFour': {'nArgs':4, 'PATTERN':'   * - | _ARG0_\n'+'     - | _ARG1_\n'+'     - | _ARG2_\n'+'     - | _ARG3_\n'},
    '\\rowTableFive': {'nArgs':5, 'PATTERN':'   * - | _ARG0_\n'+'     - | _ARG1_\n'+'     - | _ARG2_\n'+'     - | _ARG3_\n'+'     - | _ARG4_\n'},
    '\\startTable':{'nArgs':3, 'PATTERN':'\n.. list-table:: \\ \n   :widths: auto\n   :header-rows: 1\n', 'type':'TABLE'}, 
    
    '\\refSectionA':{'nArgs':1, 'PRE':' :ref:`Section <','POST':'>`\\ ', 'type':'REF'},
    '\\refSection':{'nArgs':1, 'PRE':'Section :ref:`','POST':'`\\ ', 'type':'REF'},
    '\\refChapter':{'nArgs':1, 'PRE':'Section :ref:`','POST':'`\\ ', 'type':'REF'},

    '\\exuUrl':{'nArgs':2, 'PATTERN':'`_ARG1_ <_ARG0_>`_', 'type':'URL'},
    '\\url':{'nArgs':1, 'PRE':'\\ `_ARG0_ <','POST':'>`_\\ ', 'type':'URL'},
    
    '\\ref':{'nArgs':1, 'PRE':' :ref:`','POST':'`\\ ', 'type':'REF'},
    '\\fig':{'nArgs':1, 'PRE':'\\ :numref:`','POST':'`\\ ', 'type':'REF'}, 

    '\\hac':{'nArgs':1, 'PRE':'\\ :ref:`_ARG0_ <','POST':'>`\\ '},
    '\\hacs':{'nArgs':1, 'PRE':'\\ :ref:`_ARG0_ <','POST':'>`\\ '},
    '\\acs':{'nArgs':1, 'PRE':'\\ :ref:`_ARG0_ <','POST':'>`\\ '},
    '\\acp':{'nArgs':1, 'PRE':'\\ :ref:`_ARG0_ <','POST':'>`\\ '},
    '\\acf':{'nArgs':1, 'PRE':'\\ :ref:`_ARG0_ <','POST':'>`\\ '},
    '\\ac':{'nArgs':1, 'PRE':'\\ :ref:`_ARG0_ <','POST':'>`\\ '},
    '\\eqref':{'nArgs':1, 'PRE':'\\ :eq:`','POST':'`\\ ', 'type':'REF'},
    '\\eqs':{'nArgs':1, 'PRE':'Eqs. :eq:`','POST':'`\\ ', 'type':'REF'},
    '\\eqq':{'nArgs':1, 'PRE':'\\ :eq:`','POST':'`\\ ', 'type':'REF'},
    '\\eq':{'nArgs':1, 'PRE':'Eq. :eq:`','POST':'`\\ ', 'type':'REF'},

    #'\\label':{'nArgs':1, 'PRE':'Eq. :eq:`','POST':'`\\ ', 'type':'LABEL'},
    '\\addExampleImage':{'nArgs':1, 'action':'REMOVE'},
}

#conversion into dictionary; action indicates usage; special functions for special structures
specialStructures = {
    '\\begin{lstlisting}':{'close':'\\end{lstlisting}', 'type':'CODE', 'action':'USE'},
    '\\begin{pytlisting}':{'close':'\\end{lstlisting}', 'type':'CODEPYTHON', 'action':'USE'},
    '\\begin{figure}':{'close':'\\end{figure}', 'type':'FIGURE', 'action':'REMOVE'},
    '\\begin{center}':{'close':'\\end{center}', 'type':'NONE', 'action':'REMOVE'},
    '\\begin{array}':{'close':'\\end{array}', 'type':'EQUATION', 'action':'KEEP'},
    '\\begin{cases}':{'close':'\\end{cases}', 'type':'EQUATION', 'action':'KEEP'},
    '\\bi':{'close':'\\ei', 'type':'LIST', 'action':'USE'},
    '\\bn':{'close':'\\en', 'type':'LIST', 'action':'USE'},
    '\\be':{'close':'\\ee', 'type':'EQUATION', 'action':'USE'},
    '\\bea':{'close':'\\eea', 'type':'EQUATION', 'action':'USE'},
    }

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
testText = r"""
%abc
\mysubsubsection{What is \codeName ?}
\codeName -- (fl\mybold{EX}ible m\mybold{U}ltibody \mybold{DYN}amics  -- \mybold{EX}tend yo\mybold{U}r \mybold{DYN}amics) \vspace{6pt} \\ 
$$
  x=1+2
$$
\noindent \codeName is a $x+y$ C++ based Python library for efficient simulation of flexible multibody dynamics systems.
It is the follow up code of the previously developed multibody code HOTINT, which Johannes Gerstmayr started during his PhD-thesis.
It seemed that the previous code HOTINT reached limits of further (efficient) development and it seemed impossible to continue from this code as it was outdated regarding programming techniques and the numerical formulation at the time \codeName was started.

\codeName is designed to easily set up complex multibody models, consisting of rigid and flexible bodies with joints, loads and other components. It shall enable automatized model setup and parameter variations, which are often necessary for system design but also for analysis of technical problems. The broad usability of Python allows to couple a multibody simulation with environments such as optimization, statistics, data analysis, machine learning and others.

\begin{lstlisting}
  #this is code
  print(1+1)
\end{lstlisting}

\begin{figure}
  this is stupid figure text \includegraphics[asdf]
\end{figure}

%\bn
%  \item This is item 1
%  \bi
%    \item level 2 item
%  \ei
%%\item This is item void with $x+y=z$
%\item This is item 3 with $x+y=z$
%\onlyRST{\rstStartNewLine}
%%
%\bi
%\item level 2b item
%\ei
%\en
%\bi
%  \item new item
%\ei
The multibody formulation is mainly based on redundant coordinates. This means that computational objects (rigid bodies, flexible bodies, ...) are added as independent bodies to the system. Hereafter, connectors (e.g., springs or constraints) are used to interconnect the bodies. The connectors are using Markers on the bodies as interfaces, in order to transfer forces and displacements.
For details on the interaction of nodes, objects, markers and loads see \refSection{sec:overview:items}. For a non-redundant formulation, see \texttt{ObjectKinematicTree} -- this allows to create tree-structures with minimal coordinates in \codeName.

\bn
\item abc
\bi
\item[$\ra$] Item2
\begin{lstlisting}
asdf
\end{lstlisting}
\item[$\ra$] Item3
\ei
\en
"""

#class which transforms latex text into a list of dictionaries
# types are: 'NONE', 'EQUATION', 'EQUATION$', 'EQUATION$$', 'COMMAND', 'LABEL', 'CODE', 'CODEPYTHON', 'LIST', 'ENUMERATION', 'RST'
class LatexConverter:
    def __init__(self):
        # self.latexText = latexText
        self.keyCharacters = '\\$%'

        self.commandDelimiters = commandDelimiters
        
        self.commands = {}
        for key, value in noArgCommands.items():
            self.commands[key] = 0

        for key, value in multiArgCommands.items():
            self.commands[key] = multiArgCommands[key]['nArgs']


        # print(self.commands)
        self.specialCommands = {}
        self.specialCommands['\\begin'] = 1
        # self.specialCommands['be'] = None
        # self.specialCommands['refSection'] = 1
        
        for key, value in specialStructures.items():
            if '\\begin' not in key:
                self.specialCommands[key] = 0

                
        # if latexText:
        #     self.data = self.Convert()
        # print()
    
        
    #read command after having read the first "\" character
    def ReadCommand(self, text, i):
        endCommand = False
        command = ''
        j = i
        
        stillBackslash = True #for multiple backslashes
        goOn = True
        
        #j<=i is for "\,", "\ ", "\\", etc.
        while j < len(text) and goOn and (j <= i or (text[j] not in self.commandDelimiters) or (stillBackslash and text[j] == '\\')):
            command += text[j]
            if text[j] != '\\' and text[j] in self.commandDelimiters and j == i: 
                goOn = False
            else:
                j += 1
                if text[j] != '\\': 
                    stillBackslash=False
        
        # print('j=',j,' command=',command)
        return command

    #some text, which is always replaced
    def PrepText(self, text):
        t = text
        for key,value in directConversion.items():
            t = t.replace(key,value)
        return t
    
    #read simple {abc} arg, no hierarchical braces!
    def ReadSimpleArg(self, text, i):
        start = SafeFind(text, '{',i)
        if start == -1: raise ValueError('ReadSimpleArg: no open brace found: '+text[i-20:i+20])
        end = SafeFind(text, '}',start+1)
        if end == -1: raise ValueError('ReadSimpleArg: no closing brace found: '+text[i-20:i+20])

        return text[start+1:end]

    #convert into list between i and end; consider hierarchies up to level 2
    def ConvertList(self, text, i, end):
        itemText = '\\item'
        found = True
        n = len(itemText)
        listSplit = []
        j = i

        while found:
            pos = SafeFind(text,itemText,j,end)

            foundOther = True
            while foundOther:
                foundOther = False

                posOther = SafeFind(text,'\\bi',j,end)
                if posOther != -1 and posOther < pos:
                    # print('** FOUND SECOND bi **')
                    foundOther = True
                    endOther = FindMatchingClosingText(text, posOther+3, openText='\\bi',
                                            closingText='\\ei')
                    pos = SafeFind(text,itemText,endOther,end)
                    j = endOther+3
                else:
                    posOther = SafeFind(text,'\\bn',j,end)
                    if posOther != -1 and posOther < pos:
                        # print('** FOUND SECOND bn **')
                        foundOther = True
                        endOther = FindMatchingClosingText(text, posOther+3, openText='\\bn',
                                                closingText='\\en')
                        pos = SafeFind(text,itemText,endOther,end)
                        j = endOther+3
                
            if pos == -1: 
                found = False
            else:
                # print(text[pos:pos+20])
                if pos+n < end-1:
                    if text[pos+n] in self.commandDelimiters:
                        listSplit += [pos]
                j = pos+n
        
        if len(listSplit) == 0: raise ValueError('ConvertList: list with no \\item:'+text[i:i+40])

        listSplit += [end]
        
        listText = []
        last = listSplit[0]+n
        for ind in listSplit[1:]:
            # if '\\item' in text[last:ind]:
            #     print('*****\nitem in:'+text[last:ind])
            listText += [self.ConvertText(text, last, ind)]
            last = ind+n

        # print('listText=',listText)
        return listText

    #convert text from start to end into list of dictionaries and return list
    def ConvertText(self, text, startIndex, endIndex, mode='NONE'):
        global listUnknownCommands
        # print('mode=',mode)
        i = startIndex
        t = ''
        data = []
        while i < endIndex:
            # print('at:"'+text[i:i+10].replace('\n','\\n')+'"')
            if text[i] not in self.keyCharacters or mode.startswith('CODE') or mode=='RST' or mode=='REMOVE':
                t += text[i]
                i += 1
            else:
                if t != '': data += [t]
                t = ''
                if text[i] == '$':
                    if text[i+1] == '$':
                        i+=2
                        newEnd = SafeFind(text,'$$',i,optional=False)
                        data += [{'type':'EQUATION$$', 'body':self.ConvertText(text, i, newEnd)}]
                        i=newEnd+2
                    else:
                        i+=1
                        newEnd = SafeFind(text,'$',i,optional=False)
                        data += [{'type':'EQUATION$', 'body':self.ConvertText(text, i, newEnd)}]
                        i=newEnd+1
                elif text[i] == '\\':
                    #read command:
                    command = '\\'+self.ReadCommand(text, i+1)
                    i+=len(command) #+'\'
                    if command in self.specialCommands:
                        nArgs = self.specialCommands[command]
                        #read simple arg like in \begin{...} or \ref{...}
                        arg = ''
                        commandText = command
                        if nArgs:
                            arg = self.ReadSimpleArg(text, i)
                            i += 2+len(arg)
                            i = SafeFind(text,'\n',i,optional=False) #code starts after end of line
                            commandText += '{'+arg+'}'
                        
                        if commandText not in specialStructures: 
                            raise ValueError('Did not find specialStructure: ', commandText)
                            
                        struct = specialStructures[commandText]
                        end = FindMatchingClosingText(text, i, openText=commandText,
                                                closingText=struct['close'])
                        
                        #print('open=',commandText, ', close=',struct['close'], ', text='+text[i:end]+'\n')
                        d = {'command': commandText}

                        if struct['type'] == 'CODE' or struct['action'] == 'REMOVE':
                            # print('CODE:', i, end)
                            d['body'] = text[i:end]
                        elif struct['type'] == 'LIST':
                            d['body'] = self.ConvertList(text, i, end)
                        else:
                            d['body'] = self.ConvertText(text, i, end, 'NONE')
                            
                        for key, value in struct.items():
                            d[key] = copy(value)
                            
                        data += [d]

                        i = end+len(struct['close'])
                        
                        # if command == 'begin' and arg == 'lstlisting':
                        #     end = FindMatchingClosingText(text, i, openText='\\begin{lstlisting}',
                        #                             closingText='\\end{lstlisting}')
                        #     data += [{'type':'CODE', 'body':text[i:end]}]
                        #     i = end+len('\end{lstlisting}')
                            
                    elif command in keepCommands:
                        # print('keep command:', command)
                        t += command
                        i += len(command)
                    elif command not in self.commands:
                        if command not in listUnknownCommands:
                            listUnknownCommands += [command]
                            print('Invalid command: "'+command+'" in text: '+text[i-20:i+20])
                        t += command
                        i += len(command)
                        # raise ValueError('Invalid command: "'+command+'" in text: '+text[i-20:i+20])
                    else:
                        nArgs = self.commands[command]
                        
                        # if nArgs:
                        #     d = copy(multiArgCommands['\\'+command])
                        # else:
                        #     d = (noArgCommands['\\'+command])
                        # print(d)
                        d={'nArgs':nArgs}
                        d['command'] = command

                        for ia in range(nArgs):
                            #print('search',nArgs,'args in '+command)
                            [i,newEnd] = FindMatchingBracket(text, SafeFind(text,'{',i,optional=False))
                            newMode = 'NONE'
                            if command in multiArgCommands:
                                struct = multiArgCommands[command]
                                # print('command:'+command+ ', struct:',struct)
                                if 'action' in struct:
                                    if struct['action']=='REMOVE' or struct['action']=='RST':
                                        newMode = struct['action']
                            d['ARG'+str(ia)] = self.ConvertText(text, i+1, newEnd, newMode)
                            #print('found:', text[i+1:newEnd],'==',lArgs[-1])
                            i = newEnd+1
                        data += [d]
                elif text[i] == '%': #\% is a command!
                    lasti = i
                    i = text.find('\n',i+1)+1 #don't add comments to data!
                    #print('comment:'+text[lasti:i])
                else: raise ValueError('invalid key:"'+text[i]+'"')
        if t != '':
            # data += [{'type':mode, 'body':t}]
            data += [t]

        return data

    def Convert(self, latexText):
        text = self.PrepText(latexText)

        if '\t' in text: print('\n\nWARNING: text contains TABs!\n\n')

        i = 0 #string index
        data = self.ConvertText(text, 0, len(text), 'NONE')
        return data
    
    #List2RST
    #Equation2RST
    #reuse label and section functions
    #add option for readable text
    
    
    #convert data, previously converted with Convert() into RST format
    def ToRST(self, data):
        text = ''
        for item in data:
            if type(item) == str:
                text += item
        return text
                
        
#%%++++++++++++++++++++++++++++++++++++++++++
#load file

files = [
        #'itemDefinition.tex',
        # 'version.tex',
        'gettingStarted.tex',
        # 'introduction.tex',
        # 'tutorial.tex',
        # 'GUI.tex',
        # 'notation.tex',
        # 'theory.tex',
        # 'solver.tex',
        ]

testText = ''
for filename in files:
    with open('../../../docs/theDoc/'+filename, 'r', encoding='utf-8') as file:
        testText += file.read()

converter = LatexConverter()
data = converter.Convert(testText)

with open('test.txt', 'w', encoding='utf-8') as file:
    for item in data:
        file.write(str(item)+'\n')

print(converter.ToRST(data))
        
#print(LatexConverter(testText).data)
    



