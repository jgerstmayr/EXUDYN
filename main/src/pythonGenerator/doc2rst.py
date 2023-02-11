# -*- coding: utf-8 -*-
"""
Created on Tue Jun 09 23:53:30 2020

@author: Johannes Gerstmayr

goal: generate .rst files from tex documentation

usage: call 'rstviewer README.rst' directly in powershell; alternatively use restview
"""
import copy #for deep copies
from autoGenerateHelper import Str2Latex, GenerateLatexStrKeywordExamples, ExtractExamplesWithKeyword

sectionFilesDepth = 1 #0=best for pydata theme, 1=best for read the docs theme
reducedREADME = True #True, if Sphinx anyway used
createIndexHTML = False #only needed, if Sphinx not used
createSphinxFiles = True
sectionMarkerText = '%%SECTIONLEVEL' #add level: 0,1,2, ...

rstFolder = 'docs/RST/' #folder where generated .rst files are stored

sourceDir='../../../docs/theDoc/'
destDir='../../../'
filesParsed=[
              'version.tex',
              'buildDate.tex',
              'gettingStarted.tex',
              'introduction.tex',
              'tutorial.tex',
             ]

convWords={'(\\the\\month-\\the\\year)':'',
           '    \item':'\item',
           '  \item':'\item',
           '\item[$\\ra$]':'  |  => ', #'+ ->', #this is always a sublist
           '\\item[]':'  ', 
           '\\item[--]':' - ',  #one additional whitespace at beginning for alignment of sub-lists!
           '\\item':'+ ',
           '\\small':'',
           '\\noindent ':'',
           '\\noindent':'',
           '$\\ra$':'=>',
           '\\newpage':'',
           '\\horizontalRuler':'',
           '\\\\':'\n\n',
           '$\\backslash$':'\\',
           '\\plainlststyle':'',
           '\\codeName\\':'Exudyn',
           '\\codeName':'Exudyn',
           '\\pythonstyle\\begin{lstlisting}':'\n.. code-block:: python\n',
           '\\begin{lstlisting}':'\n.. code-block::\n',
           #'\\end{lstlisting}':'\ \n',#gives errors with restview.exe
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
convCommands={#(precommand,'_USE'/'',postcommand)
    '\\ignoreRST':('','',''),
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
    '\\footnote':(' (','_USE',')'),
    '\\mybold':('\\ **','_USE','**\\ '),
    '\\mathrm':('','_USE',''),
    '\\cite':('','',''),
    '\\onlyRST':('','_USE',''),
    '\\refSection':('theDoc.pdf','',''),
    '\\fig':('[figure in theDoc.pdf]','',''),
    '\\exuUrl':('`','_USE','`_'),
    '\\ref':('[theDoc.pdf]','',''),
    'figure':('','',''),
    } #TITLE, SUBTITLE, SUBSUBTITLE, ...

sectionsList = []

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
    global sectionsList
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
                innerString = ReplaceWords(innerString, convWords) #needs to be cleaned here already
                if key == '\\mysection':
                    s += sectionMarkerText+'0\n'
                    sectionsList += [('0',innerString)]
                    s += '\n'+'='*len(innerString)+'\n'
                    s += innerString+'\n'
                    s += '='*len(innerString)+'\n'
                elif key == '\\mysubsection':
                    if sectionFilesDepth > 0:
                        s += sectionMarkerText+'1\n'
                        sectionsList += [('1',innerString)]
                    s += '\n'+'-'*len(innerString)+'\n'
                    s += innerString+'\n'
                    s += '-'*len(innerString)+'\n'
                elif key == '\\mysubsubsection':
                    if sectionFilesDepth > 1:
                        s += sectionMarkerText+'2\n'
                        sectionsList += [('2',innerString)]
                    s += innerString+'\n'
                    s += '='*len(innerString)+'\n'
                elif key == '\\mysubsubsubsection':
                    if sectionFilesDepth > 2:
                        s += sectionMarkerText+'3\n'
                        sectionsList += [('3',innerString)]
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

#extract single .rst file and hierarchical files with sections, subsections, etc.
#marked with %%SECTION %%SUBSECTION 
#can be called recursively to get subsections (level1=sub, level2=subsub, ...)
def ExtractSections(rstStringWithMarkers):

    hierarchicalRST = [] #collect lists
    
    found = 0 #starting point for search
    iStart = 0
    iEnd = 0
    level = 0

    foundAtCurrentLevel = False #only false at start
    endFound = False
    
    while not endFound:
        found = rstStringWithMarkers.find(sectionMarkerText, found)
        if found == -1: #no further sections => store remaining text at current level
            #print('no more found, take rest of file')
            endFound = True
            found = len(rstStringWithMarkers) #end of file
            hierarchicalRST += [(level, rstStringWithMarkers[iStart:found])]
        else:
            foundStart = found
            found += len(sectionMarkerText)
            nextLevel = int(rstStringWithMarkers[found])
            #print('nextLevel=', nextLevel, ', pos=', found)
            found += 2 #go after level number + '\n'

            if nextLevel != level:
                #finish previous level:
                hierarchicalRST += [(level, rstStringWithMarkers[iStart:foundStart])]
                #start next level
                level = nextLevel
                foundAtCurrentLevel=True
                iStart = found #omit marker text
            elif foundAtCurrentLevel:
                hierarchicalRST += [(level, rstStringWithMarkers[iStart:foundStart])]
                iStart = found #omit marker text
            else:
                foundAtCurrentLevel=True #do not store, but do further processing for this level
    
    for i in range(sectionFilesDepth+1):
        rstStringWithMarkers = rstStringWithMarkers.replace(sectionMarkerText+str(i)+'\n','')

    return [rstStringWithMarkers, hierarchicalRST]

#create valid file name (erase spaces, ?, -, ...)
#restrict to 16 characters
def SectionNameToFileName(sectionName):
    sNew = ''
    for i in range(len(sectionName)):
        c = sectionName[i]
        if i > 0:
            if sectionName[i-1] == ' ':
                c = c.upper()
        sNew += c
    
    s = sNew.replace(' ','').replace('?','').replace('-','').replace('+','').replace(':','').replace(',','').replace('.','')
    return s[0].upper() + s[1:]

def ConvertFile(s):
    s=s.replace('\\ ',' ') #replace special spaces from latex first; spaces that are added later shall be kept!
    s=ReplaceCommands(s, convCommands)
    s=ReplaceWords(s, convWords)
    
    return s

print('------------------------------------------')
print('converting latex docu into README.rst file...')

sHEADERsmall = """|PyPI version exudyn| |PyPI pyversions| |PyPI download month|

.. |PyPI version exudyn| image:: https://badge.fury.io/py/exudyn.svg
   :target: https://pypi.python.org/pypi/exudyn/

.. |PyPI pyversions| image:: https://img.shields.io/pypi/pyversions/exudyn.svg
   :target: https://pypi.python.org/pypi/exudyn/

.. |PyPI download month| image:: https://img.shields.io/pypi/dm/exudyn.svg
   :target: https://pypi.python.org/pypi/exudyn/

"""
sHEADER = '|Documentation GithubIO| '+sHEADERsmall+"""

.. |Documentation GithubIO| image:: https://img.shields.io/website-up-down-green-red/https/jgerstmayr.github.io/EXUDYN.svg
   :target: https://jgerstmayr.github.io/EXUDYN/

"""
# #unused / does not work:
# abc="""
# |Github all releases|

# .. |Github all releases| image:: https://img.shields.io/github/downloads/jgerstmayr/EXUDYN/total.svg
#    :target: https://GitHub.com/jgerstmayr/EXUDYN/releases/

# |GitHub commits|

# .. |GitHub commits| image:: https://img.shields.io/github/commits-since/jgerstmayr/EXUDYN/v1.0.0.svg
#    :target: https://GitHub.com/jgerstmayr/EXUDYN/commit/

# |PyPI download total|

# .. |PyPI download total| image:: https://img.shields.io/pypi/dt/exudyn.svg
#    :target: https://pypi.python.org/pypi/exudyn/   

# |GitHub commits|

# .. |Github all releases| image:: https://img.shields.io/github/commits-since/jgerstmayr/EXUDYN/v1.5.0.svg
#    :target: https://github.com/jgerstmayr/EXUDYN/commit/

# """

sRST = ''
sFile = ''
#sFile += '======\nExudyn\n======\n' #add header for .rst file
sFile += '\mysection{Exudyn}\n' #add header for .rst file
sFile += '\n**A flexible multibody dynamics systems simulation code with Python and C++**\n\n'

for fileName in filesParsed:
    fileLines = []
    file=open(sourceDir+fileName,'r') 
    fileLines = file.readlines()
    file.close()

    nLines = len(fileLines)
    
    for line in fileLines:
        if not (len(line.strip()) > 0 and line.strip()[0] == '%'):
            sFile += line# + '\n'
    
    #print(sFile)
    if fileName == 'version.tex':
        # sRST += '\n| '
        sFile += '\n+  '
    if fileName == 'buildDate.tex':
        # sRST += '| '
        sFile += '+  '
    sRST += ConvertFile(sFile)

    sFile = ''

sRST += '\n\n\ **FOR FURTHER INFORMATION see** `Exudyn Github pages <https://jgerstmayr.github.io/EXUDYN>`_ and'
sRST += ' see `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ !!!\n\n'
# sRST += '\n\n\ **FOR FURTHER INFORMATION GO TO theDoc.pdf !!!**\ \n\n'
sRST += """.. |pic7| image:: docs/demo/screenshots/logoRST.png
   :width: 120
   
|pic7| 
"""

[sRSTmain, hierarchicalRST] = ExtractSections(sRST)

sRSTreduced = hierarchicalRST[0][1].replace(sectionMarkerText+'0','')
sRSTreduced += '\n\n\ **FOR FURTHER INFORMATION see** `Exudyn Github pages <https://jgerstmayr.github.io/EXUDYN>`_ and'
sRSTreduced += ' for details (incl. equations) see `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ !!!\n\n'

if True:
    #this is the file used by github directly
    rstFile = 'README.rst'
    
    
    file=open(destDir+rstFile,'w')  #clear file by one write access
    # file.write('% ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++')
    # file.write('% description of python utility functions; generated by Johannes Gerstmayr')
    # file.write('% ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n\n')
    if not reducedREADME:
        file.write(sHEADER+sRSTmain)
    else:
        file.write(sHEADER+sRSTreduced)
    file.close()

print('----------- finished ---------------------')

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#temporary, create single html file
if createIndexHTML:
    print('------------------------------------------')
    print('converting README.rst to index.html file...')

    htmlFile = 'index.html'
    
    import docutils.core

    docutils.core.publish_file(
        source_path = destDir+rstFile,
        destination_path = destDir+htmlFile,
        writer_name ="html")

#replace paths for figures in SPHINX:
sRST = sRST.replace('docs/theDoc/figures/','../theDoc/figures/')
sRST = sRST.replace('docs/demo/','../demo/')
[sRST, hierarchicalRST] = ExtractSections(sRST)

modMainPage = hierarchicalRST[0][1].replace(sectionMarkerText+'0','')
modMainPage += '\n\n\ **READ Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ \n\n'

hierarchicalRST[0] = (hierarchicalRST[0][0], sHEADERsmall + modMainPage)

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#temporary, create single html file
if createSphinxFiles:
    print('---------------------------')
    print('create files for SPHINX ...')

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #create index ==> NOT NEEDED
    if False:
        import re
        
        regex = re.compile('[^a-zA-Z _]') #only letters and 
        sRST1 = sRST.replace('\n',' ').replace('.',' ').replace('/',' ').replace('\\',' ').replace(',',' ').replace(';',' ').replace('-',' ')
        sRST1 = sRST1.replace('(',' ').replace(')',' ')
        sRST2 = regex.sub('', sRST1)
        listIndex = sRST2.split(' ')
    
        listIndex2 = []
        previous = ''
        for i, text in enumerate(listIndex):
            if text != '' and len(text) >= 10:
                listIndex2 += [text.lower()]
    
        listIndex2.sort()
        listIndex3 = []
        for i, text in enumerate(listIndex2):
            if text.lower() != previous:
                previous = text.lower()
                listIndex3 += [previous]
                
        print('index size=', len(listIndex3))
    
        sListIndex = ''
        for item in listIndex3:
            sListIndex += '   single: '+item+'\n'
        
    indexRST = """.. Exudyn documentation master file, 

Exudyn documentation
====================
"""

    indexRST += """
..  contents::
    :local:
    :depth: 2
"""
    #indexRST += sListIndex #need to check if this really works

    # rst/README
    # indexRST += '   '+rstFolder+'index\n'
    if len(sectionsList) != len(hierarchicalRST):
        raise ValueError('SPHINX .rst file build: illegal section structure')

    filesList = [] #filename and content tuple
    primaryTocList = []
    secondaryTocLists = []
    secondaryTocList= [] #secondary toc list
    for i, item in enumerate(hierarchicalRST):
        name = sectionsList[i][1]
        filename = rstFolder+SectionNameToFileName(name)
        level = int(sectionsList[i][0])
        # if level == 0:
        #     # if sectionFilesDepth == 0:
        #     #     indexRST += '   '+filename+'\n'
        #     print('main index: '+filename)
        #     currentMainSection = sectionsList[i][1]
        # else:
        #     indexRST += '   '+filename+'\n'
        filesList += [(filename, name, level, item[1])]

    #+++++++++++++++++++++++++
    
    #+++++++++++++++++++++++++
    if sectionFilesDepth==0:
        #create primary toc
        indexRST += """
.. toctree::
   :maxdepth: 3
   :caption: Contents:

"""
        for (filename, name, level, content) in filesList:
            if level == 0:
                indexRST += '   '+filename+'\n'
    else: #1
        #create secondary tocs (works well for ReadTheDocs theme)
        currentMainSection = ''
        for (filename, name, level, content) in filesList:
            if level == 0: #for every main section, add a new toctree
                indexRST += '\n'
                indexRST += '.. toctree::\n'
                indexRST += '   :caption: '+name+'\n'
                indexRST += '   :maxdepth: 3\n\n'
            indexRST += '   '+filename+'\n'

    indexRST += """
.. toctree::
   :caption: Issue Tracker
   
   docs/trackerlogRST

Indices and tables
==================

* :ref:`genindex`
* :ref:`search`

"""

    file=open(destDir+'index.rst','w')  #clear file by one write access
    file.write(indexRST)
    file.close()

    for (filename, name, level, content) in filesList:
        file=open(destDir+filename+'.rst','w')  #clear file by one write access
        file.write(content)
        file.close()
        

    #print('sections list:', sectionsList)
    # print('files list:', filesList)
    
    # for i, item in enumerate(sectionsList):
    #     print('WRITE: section'+item[0]+':',SectionNameToFileName(item[1]))
        


      