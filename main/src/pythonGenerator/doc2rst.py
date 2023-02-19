# -*- coding: utf-8 -*-
"""
Created on Tue Jun 09 23:53:30 2020

@author: Johannes Gerstmayr

goal: generate .rst files from tex documentation

usage: call 'rstviewer README.rst' directly in powershell; alternatively use restview
"""
import copy #for deep copies
import io   #for utf-8 encoding
from autoGenerateHelper import Str2Latex, GenerateLatexStrKeywordExamples, ExtractExamplesWithKeyword, \
                               RSTheaderString, RSTlabelString, ExtractLatexCommand, FindMatchingBracket, ReplaceWords, RSTurl, \
                               convLatexWords, convLatexCommands, ReplaceLatexCommands
                               #ReplaceLatexCommands not imported as it has a special local version

sectionFilesDepth = 1 #0=best for pydata theme, 1=best for read the docs theme
reducedREADME = True #True, if Sphinx anyway used
createIndexHTML = False #only needed, if Sphinx not used
createSphinxFiles = True
sectionMarkerText = '%%SECTIONLEVEL' #add level: 0,1,2, ...

rstFolder = 'docs/RST/' #folder where generated .rst files are stored

sourceDir='../../../docs/theDoc/'
destDir='../../../'

#main files
filesParsed=[
              'version.tex',
              'gettingStarted.tex',
              'introduction.tex',
              'tutorial.tex',
              'GUI.tex',
             ]

undefLabelList = [
    ('Theory: Contact','seccontacttheory'),
    ('List of Abbreviations','sec-listofabbreviations'),
    ##('Render State','sec-renderstate'),
    ##('GraphicsData','sec-graphicsdata'),
    ('Solvers','sec-solvers'),
    ('Items Reference Manual','sec-item-reference-manual'),
    ('Solvers: Static','sec-solver-solvestatic'),
    ('Solvers: Dynamic','sec-solver-solvedynamic'),
    ('Solvers: Eigenvalues','sec-solver-computeode2eigenvalues'),
    ('Theory: Component Mode Synthesis','sec-theory-cms'),
    #'sec-mainsolverstatic',
    ##('Graphics: UTF-8','sec-utf8'),
    ('Solver: Explicit','sec-explicitsolver'),
    ]
undefLabels =''
for i, (header, label) in enumerate(undefLabelList): 
    undefLabels+='.. _'+label+':\n\n'+header+'\n'+'='*len(header)+'\n\n'
    undefLabels+='See according section in `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ \n\n'
    
undefLabels+='Further information\n'
undefLabels+='===================\n\n'
undefLabels+='\ **SEE Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ '

# sectionsList = []
# #local copy, will do some extra things and stores sections
# def ReplaceLatexCommands(s, conversionDict): #replace strings provided in conversion dict
#     global sectionsList
#     s = s.replace('{\\bf ','\\mybold{') #this is then further converted into rst code ...
#     for (key,value) in conversionDict.items():
#         cnt = 0
#         found = 0
#         while (found != -1):
#             secondBracket = 0
#             isBeginEnd = False
#             if len(value) > 3:
#                 secondBracket = len(value)-3
#             if key == 'figure': isBeginEnd = True

#             found = ExtractLatexCommand(s, key, secondBracket, isBeginEnd)
#             if found != -1:
#                 [preString, innerString, innerString2, postString] = found

#                 s = preString
#                 if '\\refSection' in key  or key == '\\label' or key == '\\fig' or key == '\\ref':
#                     innerString=innerString.replace(':','-').replace('_','-').lower()
#                 elif value[1] == '_USE' and key != '\\exuUrl':
#                     innerString = ReplaceLatexCommands(innerString, convLatexCommands)
#                     innerString = ReplaceWords(innerString, convLatexWords) #needs to be cleaned here already

#                 if '\\rowTable' in key:
#                     nRows = len(value)-2
                    
#                     #print('rowTableThree/Four: rows=',nRows)
#                     if type(innerString2) == list:
#                         # if len(innerString2) != nRows:
#                         #     print('innerString2:',innerString2)
#                         text = ''
#                         cstar = '*'
#                         for k, col in enumerate(innerString2):
#                             #seems that it needs to have exactly same amount of pre-spaces as :widths: and other options
#                             text += '   '+cstar+' - | '+col
#                             if k < len(innerString2)-1:
#                                 text += '\n' #last \n is added due to text itself (postString)
#                             cstar = ' '
#                         s += text
#                         #print('table = \n'+text)
#                     else:
#                         print('PROBLEM with rowTable: ',innerString2)
# # * - **HEADER1**
# #    - **HEADER2**
# #    - **HEADER3**
# #  * - | TEXT 1
# #    - | MULTILINE                             
#                 elif key == '\\mysection' or key == '\\mysectionlabel':
#                     s += sectionMarkerText+'0\n'
#                     if 'label' in key: s += RSTlabelString(innerString2)+'\n'
#                     sectionsList += [('0',innerString)]
#                     s += '\n'+RSTheaderString(innerString, 1)
#                 elif key == '\\mysubsection' or key == '\\mysubsectionlabel':
#                     if sectionFilesDepth > 0:
#                         s += sectionMarkerText+'1\n'
#                         sectionsList += [('1',innerString)]
#                     if 'label' in key: s += RSTlabelString(innerString2)+'\n'
#                     s += '\n'+RSTheaderString(innerString, 2)
#                 elif key == '\\mysubsubsection' or key == '\\mysubsubsectionlabel':
#                     if sectionFilesDepth > 1:
#                         s += sectionMarkerText+'2\n'
#                         sectionsList += [('2',innerString)]
#                     if 'label' in key: s += RSTlabelString(innerString2)+'\n'
#                     s += '\n'+RSTheaderString(innerString, 3)
#                 elif key == '\\mysubsubsubsection' or key == '\\mysubsubsubsectionlabel':
#                     if sectionFilesDepth > 2:
#                         s += sectionMarkerText+'3\n'
#                         sectionsList += [('3',innerString)]
#                     if 'label' in key: s += RSTlabelString(innerString2)+'\n'
#                     s += '\n'+RSTheaderString(innerString, 4)
#                 elif key == '\\exuUrl':
#                     s += value[0]
#                     s += innerString2 + ' <'
#                     s += innerString
#                     s += '>'
#                     s += value[2]
#                 else:
#                     s += value[0]
#                     if value[1] == '_USE':
#                         s += innerString
#                     s += value[2]
                
#                 s += postString
#     return s


sectionsList = []

#extract single .rst file and hierarchical files with sections, subsections, etc.
#marked with %%SECTION %%SUBSECTION 
#can be called recursively to get subsections (level1=sub, level2=subsub, ...)
def ExtractSections(rstStringWithMarkers):
    global sectionsList
    hierarchicalRST = [] #collect lists
    
    sectionsList = []
    found = 0 #starting point for search
    iStart = 0
    iEnd = 0
    level = 0

    foundAtCurrentLevel = False #only false at start
    endFound = False
    
    while not endFound:
        found = rstStringWithMarkers.find(sectionMarkerText, found)
        
        #print('hierarchical found:',rstStringWithMarkers[found:found+len(sectionMarkerText)+1])
        if found == -1: #no further sections => store remaining text at current level
            #print('no more found, take rest of file')
            endFound = True
            found = len(rstStringWithMarkers) #end of file
            hierarchicalRST += [(level, rstStringWithMarkers[iStart:found])]
        else:
            [sStart, sEnd] = FindMatchingBracket(rstStringWithMarkers, found+len(sectionMarkerText), 
                                                 openBracket='[', closingBracket=']')
            [sStart2, sEnd2] = FindMatchingBracket(rstStringWithMarkers, sEnd+1,
                                                 openBracket='[', closingBracket=']')
            sNextLevel = rstStringWithMarkers[sStart+1: sEnd]
            sNextSection = rstStringWithMarkers[sStart2+1: sEnd2]
            # print('level=', sNextLevel )
            # print('Section=', sNextSection )

            sectionsList += [(sNextLevel, sNextSection)]

            #nextLevel = int(rstStringWithMarkers[found])
            
            foundStart = found
            found = sEnd2+1
            #found += len(sectionMarkerText)
            nextLevel = int(sNextLevel)
            
            #nextLevel = int(rstStringWithMarkers[found])
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
    s=s.replace(r'\ ',r' ') #replace special spaces from latex first; spaces that are added later shall be kept!
    
    s = s.replace(r'\pythonstyle\begin{lstlisting}',r'\begin{pytlisting}')

    s=ReplaceLatexCommands(s, convLatexCommands, sectionMarkerText)
    # print('count begin lstlisting=',s.count(r'\begin{lstlisting}'))
    # print('count begin pytlisting=',s.count(r'\begin{pytlisting}'))
    s=ReplaceWords(s, convLatexWords, replaceBraces=False)
    
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

#read single file; use header; erase comment lines
def ParseFile(fileName, header = ''):
    sFile = header
    fileLines = []
    file=open(fileName,'r') 
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
    return sFile

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
sRST = ''
sFile = ''
#sFile += '======\nExudyn\n======\n' #add header for .rst file
sFile += '\\mysection{Exudyn}\n' #add header for .rst file
sFile += '\n**A flexible multibody dynamics systems simulation code with Python and C++**\n\n'

for fileName in filesParsed:
    sFile = ParseFile(sourceDir+fileName, sFile)
    sRST += ConvertFile(sFile)

    sFile = ''

# print('sectionsList:\n',sectionsList)

# sRST += '\n\n\ **FOR FURTHER INFORMATION see** `Exudyn Github pages <https://jgerstmayr.github.io/EXUDYN>`_ and'
# sRST += ' see `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ !!!\n\n'
# sRST += '\n\n\ **FOR FURTHER INFORMATION GO TO theDoc.pdf !!!**\ \n\n'

##only for themes with logos:
# sRST += """.. |pic7| image:: docs/demo/screenshots/logoRST.png
#    :width: 120
#   
#|pic7| 
#"""

if False: #for debug:
    file=io.open(destDir+'test.txt','w',encoding='utf8')
    file.write(sRST)
    file.close()


[sRSTmain, hierarchicalRST] = ExtractSections(sRST)

sRSTmain += '\n\n\ **FOR FURTHER INFORMATION see** `Exudyn Github pages <https://jgerstmayr.github.io/EXUDYN>`_ and'
sRSTmain += ' see `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ !!!\n\n'

sRSTreduced = hierarchicalRST[0][1].replace(sectionMarkerText+'0','')
sRSTreduced += '\n\n\ **FOR FURTHER INFORMATION see** `Exudyn Github pages <https://jgerstmayr.github.io/EXUDYN>`_ and'
sRSTreduced += ' for details (incl. equations) see `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ !!!\n\n'

if True:
    #this is the file used by github directly
    rstFile = 'README.rst'
    
    
    file=io.open(destDir+rstFile,'w',encoding='utf8')  #clear file by one write access
    # file.write('% ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++')
    # file.write('% description of python utility functions; generated by Johannes Gerstmayr')
    # file.write('% ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n\n')
    if not reducedREADME:
        file.write(sHEADER+sRSTmain)
    else:
        file.write(sHEADER+sRSTreduced)
    file.close()

print('----------- finished ---------------------')

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

sRST = sRST.replace('docs/theDoc/figures/','../theDoc/figures/')
sRST = sRST.replace('docs/demo/','../demo/')
[sRST, hierarchicalRST] = ExtractSections(sRST)

modMainPage = hierarchicalRST[0][1].replace(sectionMarkerText+'0','')
modMainPage += '\n\n\\ **READ Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ \n\n'

hierarchicalRST[0] = (hierarchicalRST[0][0], sHEADERsmall + modMainPage)

#+++++++++++++++++++++++++++++++++++++++
#temporary, create single html file
if createSphinxFiles:
    print('---------------------------')
    print('create files for SPHINX ...')

    #take care that the Exudyn documentation heading is "higher" than that of sub-toctree index files
    indexRST = """.. Exudyn documentation master file, 

====================
Exudyn documentation
====================
"""

    #furo theme complains about contents:
#     indexRST += """
# ..  contents::
#     :local:
#     :depth: 2
# """
    if len(sectionsList) != len(hierarchicalRST):
        # print('sections:\n',sectionsList)
        #print('hierarchicalData:\n')
        # for x in hierarchicalRST:
        #     print(x[1][0:50])
        raise ValueError('SPHINX .rst file build: illegal section structure')

    filesList = [] #filename and content tuple
    primaryTocList = []
    secondaryTocLists = []
    secondaryTocList= [] #secondary toc list
    for i, item in enumerate(hierarchicalRST):
        name = sectionsList[i][1]
        filename = rstFolder+SectionNameToFileName(name)
        level = int(sectionsList[i][0])
        filesList += [(filename, name, level, item[1])]

    #+++++++++++++++++++++++++
    
    #+++++++++++++++++++++++++
    #create primary toc
    indexRST += """
.. toctree::
   :maxdepth: 3
   :caption: Exudyn User Manual

"""
    for (filename, name, level, content) in filesList:
        if level == 0:
            if name != 'Exudyn': #this file has no further index (at least now)
                indexRST += '   '+filename+'Index\n'
            else:
                indexRST += '   '+filename+'\n'

# .. toctree::
#    :caption: Python utility functions
#    :maxdepth: 3
   
   # docs/RST/items/index.rst
    indexRST += """   docs/RST/cInterface/CInterfaceIndex

.. toctree::
   :caption: Reference Manual

   docs/RST/pythonUtilities/index.rst
   docs/RST/structures/StructuresAndSettingsIndex.rst

.. toctree::
   :caption: Issue Tracker
   
   docs/trackerlogRST

Indices and tables
==================

* :ref:`genindex`
* :ref:`search`

"""
#     indexRST += """
# .. list-table::
#   :widths: 15 10 30
#   :header-rows: 1
   
#   * - **HEADER1**
#     - **HEADER2**
#     - **HEADER3**
#   * - | TEXT 1
#     - | MULTILINE 
#       | TEXT
#     - | MULTILINE
#       | TEXT 2
#   * - | DEFINITION of the long item DEFINITION of the long item DEFINITION of the long item
#     - | character
#       | given
#     - | maybe not good maybe not good maybe not good maybe not good maybe not good
#       | TEXT 2
# """

    indexRST += undefLabels + '\n'
    
    file=open(destDir+'index.rst','w')  #clear file by one write access
    file.write(indexRST)
    file.close()

    for (filename, name, level, content) in filesList:
        file=io.open(destDir+filename+'.rst','w',encoding='utf8')  #clear file by one write access
        file.write(content)
        file.close()
        
    #write subindex files; this creates a hierarchical toctree!!! 
    #take care of headings, they make magic hierarchical structure
    if True: 
        #create secondary tocs (works well for ReadTheDocs theme)
        currentMainSection = ''
        fileOpen = False
        file = None
        for (filename, name, level, content) in filesList:
            if name != 'Exudyn': #this file has no further index (at least now)
                if level == 0: #for every main section, add a new toctree
                    if fileOpen:
                        file.write('\n')
                        file.close()
                    #print('write:', destDir+filename+'Index.rst')
                    file=io.open(destDir+filename+'Index.rst','w',encoding='utf8')  #clear file by one write access
                    
                    lenName = len(name)
                    s=''
                    s += '='*lenName + '\n'
                    s += name + '\n'
                    s += '='*lenName + '\n'
                    s += '\n'
                    s += '.. toctree::\n'
                    #s += '   :caption: '+name+'\n'
                    s += '   :maxdepth: 2\n\n' #0?
                    file.write(s)
                #also include level 0 file, as this is the introduction (could be renamed ...)
                s = '   '+filename.replace(rstFolder,'')+'\n' #index file is already located in rstFolder
                #print('local file=',filename.replace(rstFolder,''))
                file.write(s)

        if fileOpen:
            file.write('\n')
            file.close()


    #print('sections list:', sectionsList)
    # print('files list:', filesList)
    
    # for i, item in enumerate(sectionsList):
    #     print('WRITE: section'+item[0]+':',SectionNameToFileName(item[1]))
        




    # #++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # #create index ==> NOT NEEDED
    # if False:
    #     import re
        
    #     regex = re.compile('[^a-zA-Z _]') #only letters and 
    #     sRST1 = sRST.replace('\n',' ').replace('.',' ').replace('/',' ').replace('\\',' ').replace(',',' ').replace(';',' ').replace('-',' ')
    #     sRST1 = sRST1.replace('(',' ').replace(')',' ')
    #     sRST2 = regex.sub('', sRST1)
    #     listIndex = sRST2.split(' ')
    
    #     listIndex2 = []
    #     previous = ''
    #     for i, text in enumerate(listIndex):
    #         if text != '' and len(text) >= 10:
    #             listIndex2 += [text.lower()]
    
    #     listIndex2.sort()
    #     listIndex3 = []
    #     for i, text in enumerate(listIndex2):
    #         if text.lower() != previous:
    #             previous = text.lower()
    #             listIndex3 += [previous]
                
    #     print('index size=', len(listIndex3))
    
    #     sListIndex = ''
    #     for item in listIndex3:
    #         sListIndex += '   single: '+item+'\n'
