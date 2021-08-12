# -*- coding: utf-8 -*-
"""
Created on Tue Jun 09 23:53:30 2020

@author: Johannes Gerstmayr

goal: generate .rst files from tex documentation
"""
import copy #for deep copies
from autoGenerateHelper import Str2Latex, GenerateLatexStrKeywordExamples, ExtractExamplesWithKeyword

sourceDir='../../../docs/theDoc/'
destDir='../../../'
filesParsed=[
              'version.tex',
              'gettingStarted.tex',
              'introduction.tex',
              'tutorial.tex',
             ]


convWords={'(\\the\\month-\\the\\year)':'',
           '	\item':'\item',
           '\\item[]':'  ', 
           '\\item[--]':' - ',  #one additional whitespace at beginning for alignment of sub-lists!
           '\\item':'+ ',
           '\\small':'',
           '\\noindent ':'',
           '\\noindent':'',
           '[$\\ra$]':'  ->',
           '$\\ra$':'->',
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
           '\\bi':'', 
           '\\ei':'',
           '\\bn':'', 
           '\\en':'',
           '\\be':'', 
           '\\ee':'',
           '\\it':'',
           '\\_':'_',
           '{\"a}':'ä',
           '{\"o}':'ö',
           '{\"u}':'ü',
           '{':'',
           '}':'',
           '$':'',
           }
convCommands={#(precommand,'_USE'/'',postcommand)
    '\\texttt':('\\ ``','_USE','``\\ '),
    '\\mysection':('','',''),
    '\\mysubsection':('','',''),
    '\\mysubsubsection':('','',''),
    '\\mysubsubsubsection':('','',''),
    '\\pythonListing':('','',''),
    '\\pythonSmallListing':('','',''),
    '\\smallListing':('','',''),
    '\\myListing':('','',''),
    '\\label':('','',''),
    '\\vspace':('','',''),
    '\\footnote':('(','_USE',')'),
    '\\mybold':('\\ **','_USE','**\\ '),
    '\\mathrm':('','_USE',''),
    '\\cite':('','',''),
    '\\ignoreRST':('','',''),
    '\\onlyRST':('','_USE',''),
    '\\refSection':('theDoc.pdf','',''),
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

#start searching for { and matching } bracket, including sub-brackets
def FindMatchingBracket(s, start):
    cnt = 0
    bStart = -1
    for i in range(start,len(s)):
        if s[i] == '{':
            cnt += 1
            bStart = i
        elif s[i] == '}':
            cnt -= 1

        if bStart != -1 and cnt == 0:
            return [bStart,i]
    return [-1,-1]
        

#if key is found, return [preString, innerString, innerString2, postString], otherwise -1; '123\section{abc}456' = ['123','abc','456']
def ExtractCommand(s, key, secondBracket, isBeginEnd):
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

def ReplaceCommands(s, conversionDict): #replace strings provided in conversion dict
    for (key,value) in conversionDict.items():
        cnt = 0
        found = 0
        while (found != -1):
            secondBracket = False
            isBeginEnd = False
            if key == '\\exuUrl': secondBracket = True
            if key == 'figure': isBeginEnd = True

            found = ExtractCommand(s, key, secondBracket, isBeginEnd)
            if found != -1:
                [preString, innerString, innerString2, postString] = found
                # if isBeginEnd:
                #     print("inner="+innerString+'+++')
                s = preString
                if key == '\\mysection':
                    s += '\n'+'='*len(innerString)+'\n'
                    s += innerString+'\n'
                    s += '='*len(innerString)+'\n'
                elif key == '\\mysubsection':
                    s += '\n'+'-'*len(innerString)+'\n'
                    s += innerString+'\n'
                    s += '-'*len(innerString)+'\n'
                elif key == '\\mysubsubsection':
                    s += innerString+'\n'
                    s += '='*len(innerString)+'\n'
                elif key == '\\mysubsubsubsection':
                    s += innerString+'\n'
                    s += '-'*len(innerString)+'\n'
                elif key == '\\exuUrl':
                    s += value[0]
                    s += innerString2 + ' <'
                    s += innerString
                    s += '>'
                    s += value[2]
                else:
                    s += value[0]
                    if value[1] == '_USE':
                        s += innerString
                    s += value[2]
                
                s += postString
    return s


# def ReplaceStrings(s, conversionDict): #replace strings provided in conversion dict
#     if replaceCurlyBracket:
#         s = s.replace('{','\{')
#         s = s.replace('}','\}')
#         s = s.replace('_','\_')

#     # s = s.replace('[','\\[')
#     # s = s.replace(']','\\]')
#     s = s.replace('&','\&')
#     return s


def ConvertFile(s):
    s=ReplaceCommands(s, convCommands)
    s=ReplaceWords(s, convWords)
    
    return s

sRST = '======\nExudyn\n======\n' #add header for .rst file

for fileName in filesParsed:
    fileLines = []
    file=open(sourceDir+fileName,'r') 
    fileLines = file.readlines()
    file.close()

    nLines = len(fileLines)
    
    sFile = ''
    for line in fileLines:
        if not (len(line.strip()) > 0 and line.strip()[0] == '%'):
            sFile += line# + '\n'
    
    #print(sFile)
    sRST += ConvertFile(sFile)
    
sRST += '\n\n ** FOR FURTHER INFORMATION GO TO theDoc.pdf ** !!!\n\n'

if True:
    rstFile = 'README.rst'
    
    file=open(destDir+rstFile,'w')  #clear file by one write access
    # file.write('% ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++')
    # file.write('% description of python utility functions; generated by Johannes Gerstmayr')
    # file.write('% ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n\n')
    file.write(sRST)
    file.close()

        
        