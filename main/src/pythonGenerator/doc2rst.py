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
                               convLatexWords, convLatexCommands, ReplaceLatexCommands, LatexString2RST, \
                               RemoveIndentation, RSTcodeBlock, RSTheaderString, RSTurl, Latex2RSTlabel
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
              'notation.tex',
              'theory.tex',
              'solver.tex',
            ]

undefLabelList = [
    ('Theory: Contact','seccontacttheory'),
    #('List of Abbreviations','sec-listofabbreviations'),
    ##('Render State','sec-renderstate'),
    ##('GraphicsData','sec-graphicsdata'),
    #('Solvers','sec-solvers'),
    #('Items Reference Manual','sec-item-reference-manual'),
    #('Solvers: Static','sec-solver-solverstatic'),
    #('Solvers: Dynamic','sec-solver-solverdynamic'),
    #('Solvers: Eigenvalues','sec-solver-computeode2eigenvalues'),
    #('Theory: Component Mode Synthesis','sec-theory-cms'),
    #'sec-mainsolverstatic',
    ##('Graphics: UTF-8','sec-utf8'),
    #('Solver: Explicit','sec-explicitsolver'),
    ]
undefLabels =''
for i, (header, label) in enumerate(undefLabelList): 
    undefLabels+='.. _'+label+':\n\n'+header+'\n'+'='*len(header)+'\n\n'
    undefLabels+='See according section in `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ \n\n'
    
undefLabels+='Further information\n'
undefLabels+='===================\n\n'
undefLabels+='\ **SEE Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ '


sectionsList = []

#extract single .rst file and hierarchical files with sections, subsections, etc.
#marked with sectionMarkerText[i][name]
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
                iStart = sEnd2+1 #for first section
    
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
    s = s.replace('\\item[$\\ra$]','  |  → ')

    s = LatexString2RST(s, sectionMarkerText=sectionMarkerText)

    # s=ReplaceLatexCommands(s, convLatexCommands, sectionMarkerText)
    # s=ReplaceWords(s, convLatexWords, replaceBraces=False, replaceDoubleBS=True)
    
    return s

print('------------------------------------------')
print('converting latex docu into README.rst file...')

sHEADERsmall = """|RTD documentation| |PyPI version exudyn| |PyPI pyversions| |PyPI download month| |Github release date| 
|Github issues| |Github stars| |Github commits| |Github last commit| |CI build|

.. |PyPI version exudyn| image:: https://badge.fury.io/py/exudyn.svg
   :target: https://pypi.python.org/pypi/exudyn/

.. |PyPI pyversions| image:: https://img.shields.io/pypi/pyversions/exudyn.svg
   :target: https://pypi.python.org/pypi/exudyn/

.. |PyPI download month| image:: https://img.shields.io/pypi/dm/exudyn.svg
   :target: https://pypi.python.org/pypi/exudyn/

.. |RTD documentation| image:: https://readthedocs.org/projects/exudyn/badge/?version=latest
   :target: https://exudyn.readthedocs.io/en/latest/?badge=latest

.. |Github issues| image:: https://img.shields.io/github/issues-raw/jgerstmayr/exudyn
   :target: https://jgerstmayr.github.io/EXUDYN/

.. |Github stars| image:: https://img.shields.io/github/stars/jgerstmayr/exudyn?style=plastic
   :target: https://jgerstmayr.github.io/EXUDYN/

.. |Github release date| image:: https://img.shields.io/github/release-date/jgerstmayr/exudyn?label=release
   :target: https://jgerstmayr.github.io/EXUDYN/

.. |Github commits| image:: https://img.shields.io/github/commits-since/jgerstmayr/exudyn/v1.0.6
   :target: https://jgerstmayr.github.io/EXUDYN/

.. |Github last commit| image:: https://img.shields.io/github/last-commit/jgerstmayr/exudyn
   :target: https://jgerstmayr.github.io/EXUDYN/

.. |CI build| image:: https://github.com/jgerstmayr/EXUDYN/actions/workflows/wheels.yml/badge.svg

"""
sHEADER = sHEADERsmall
# sHEADER = '|Documentation GithubIO| '+sHEADERsmall+"""

# .. |Documentation GithubIO| image:: https://img.shields.io/website-up-down-green-red/https/jgerstmayr.github.io/EXUDYN.svg
#    :target: https://jgerstmayr.github.io/EXUDYN/

# """
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
        if line.find('%%RSTCOMPATIBLE') != -1:
            sFile += '\\mybold{For further information on this topic read}: \\exuUrl{https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf}{theDoc.pdf}\n'
            break
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

#long README.rst, not used any more:
sRSTmain += '\n\n\ **FOR FURTHER INFORMATION see** `Exudyn Github pages <https://jgerstmayr.github.io/EXUDYN>`_ and'
sRSTmain += ' see `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ !!!\n\n'

#on github, first page:
sRSTreduced = hierarchicalRST[0][1].replace(sectionMarkerText+'0','')
sRSTreduced += '\n\nChanges can be tracked in the Issue tracker, see Github pages and Read the Docs.'
sRSTreduced += '\n\n\\ **FOR FURTHER INFORMATION see** `Exudyn Github pages <https://jgerstmayr.github.io/EXUDYN>`_\\ , '
sRSTreduced += '`Read the Docs <https://exudyn.readthedocs.io/>`_ and'
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

#on github pages and readthedocs:
modMainPage = hierarchicalRST[0][1].replace(sectionMarkerText+'0','')
modMainPage += 'Changes can be tracked in the :ref:`Issue tracker <sec-issuetracker>` \n\n'
modMainPage += 'For searching on Read the Docs (especially with the search preview), add \* or ~1 / ~2 / ... to your search to search more general, e.g., FEMinter\* to search for FEMinterface, or objetfrf~3 to find ObjectFFRF. Your search preview usually finds less results than the search when pressing Enter. See also `Read the Docs documentation <https://docs.readthedocs.io/en/stable/server-side-search/syntax.html#special-queries>`_ \n\n'
modMainPage += '\\ **READ Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ \n\n'

#last commit
#https://img.shields.io/github/last-commit/jgerstmayr/exudyn
#https://img.shields.io/github/stars/jgerstmayr/exudyn
#https://img.shields.io/pypi/v/exudyn

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
   docs/RST/items/itemsIndex.rst
   docs/RST/structures/StructuresAndSettingsIndex.rst

.. toctree::
   :caption: Misc
   
   docs/RST/Abbreviations
   docs/RST/ExamplesIndex
   docs/RST/TestModelsIndex
   docs/RST/trackerlog

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
                        fileOpen = False
                    #print('write:', destDir+filename+'Index.rst')
                    file=io.open(destDir+filename+'Index.rst','w',encoding='utf8')  #clear file by one write access
                    fileOpen = True
                    
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

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#examples and test models
if True:
    import sys, os
    sys.path.append(os.path.join(os.path.dirname(__file__), '../..', 'pythonDev/TestModels'))
    rstDir = destDir+rstFolder
    from runTestSuiteRefSol import TestExamplesReferenceSolution

    examplesTestRefSol = TestExamplesReferenceSolution()
    folderSource = '../../pythonDev/'

    #++++++++++++++++++++++++++++++++++++++
    #CREATE lists
    testFileList=[] #automatically create list from reference solution ...
    for key in examplesTestRefSol.keys():
        testFileList+=[key]

    from os import listdir
    from os.path import isfile, join

    dirPath = '../../pythonDev/Examples/'
    examplesFileList = [f for f in listdir(dirPath) if isfile(join(dirPath, f)) and '.py' in f]
    
    fileLists={'TestModels':testFileList,
               'Examples':examplesFileList}

    listTypes = ['TestModel', 'Example']
    #listTypes = ['Example']
    for fileType in listTypes:
        fileTypeS = fileType+'s'
        fileList = fileLists[fileTypeS]

        rstModelIndex = '\n'+'.. _sec-'+fileType.lower()+'s-index:'+'\n\n'
    
        rstModelIndex += RSTheaderString(fileTypeS,0)+'\n'
        rstModelIndex += 'This section includes all '+fileTypeS+' for Exudyn.'
        rstModelIndex += 'They can also be found and downloaded at the '
        rstModelIndex += RSTurl(fileTypeS+' folder of Exudyn on Github', 
                                'https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/'+fileTypeS)
        rstModelIndex += """

.. toctree::
   :maxdepth: 2
    
"""
    
        for fileName in fileList:
            #print('PROCESS '+fileType+ ' ' + fileName)
            pureName = fileName.replace('.py','')
            s=''
            with open(folderSource+fileTypeS+'/'+fileName) as f:
                s = f.read()
                
            sRST = ''
            #sRST += RSTheaderString(fileType+': '+pureName,1)+'\n'
            sRST += RSTlabelString(Latex2RSTlabel(fileTypeS+'-'+pureName))+'\n'
            sRST += RSTheaderString(fileName,1)+'\n'
    
            sRST += 'You can view and download this file on Github: '
            sRST += RSTurl(fileName, 
                          'https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/'+fileTypeS+'/'+fileName)+'\n\n'
    
            sRST += RSTcodeBlock(s, typeString='python', addLineNumbers=True, indentation='   ') + '\n'
            
            file=io.open(rstDir+fileTypeS+'/'+pureName+'.rst','w',encoding='utf8')  #clear file by one write access
            file.write(sRST)
            file.close()
    
            rstModelIndex += '   '+fileTypeS+'/'+pureName+'\n'
    
    
        file=io.open(rstDir+fileTypeS+'Index.rst','w',encoding='utf8')  #clear file by one write access
        file.write(rstModelIndex+'\n')
        file.close()




#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#create abbreviations for latex and RST:
abbrvDict={
    '2D':'two dimensions or planar',
    '3D':'three dimensions or spatial',
    'abs':'absolute (e.g., absolute error), absolute value',
    'AE':'algebraic equations',
    'CMS':'component mode synthesis',
    'coeffs':'coefficients',
    'COM':'center of mass',
    'DOF':'degree of freedom',
    'EOM':'equations of motion',
    'EP':'Euler parameters',
    'FFRF':'floating frame of reference formulation',
    'HT':'homogeneous transformation',
    'LHS':'Left-Hand-Side (of equation)',
    'LTG':'local-to-global',
    'mbs':'multibody system',
    'min':'minimum',
    'max':'maximum',
    'ODE':'ordinary differential equation',
    'ODE1':'first order ordinary differential equations',
    'ODE2':'second order ordinary differential equations',
    'pos':'position',
    'quad':'quadrangle, polygon with 4 vertices',
    'rel':'relative (e.g., relative error)',
    'RHS':'Right-Hand-Side (of equation)',
    'Rot':'rotation',
    'Rxyz':'rotation parameterization: consecutive rotations around x, y and z-axis (Tait-Bryan)',
    'STL':'STereoLithography',
    'T66':'Pl\\"ucker transformation',
    'trig':'triangle',
}

abbrvTex = ''
abbrvRST = """
.. _sec-listofabbreviations:

=====================
List of Abbreviations
=====================

This section shows typical abbreviations. For further notation, 
see also :ref:`Section Notation <sec-generalnotation>`\ .

"""
for key, value in abbrvDict.items():
    abbrvTex += '\\acro{'+key+'}{'+value+'}\n' 
    abbrvRST += '.. _'+key+':\n\n'
    s=ReplaceWords(value, convLatexWords)
    abbrvRST += '\\ **'+key+'**\\ : '+ s + '\n\n'


file=open(sourceDir+'abbreviations.tex','w')  
file.write(abbrvTex)
file.close()

#rst files should be utf8 for special characters
file=io.open(destDir+rstFolder+'Abbreviations.rst','w',encoding='utf8')  
file.write(abbrvRST)
file.close()




