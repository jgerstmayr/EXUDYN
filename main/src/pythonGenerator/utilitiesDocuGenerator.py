# -*- coding: utf-8 -*-
"""
Created on Tue Jun 09 23:53:30 2020

@author: Johannes Gerstmayr

goal: generate latex documentation for all utilities packages
"""
import copy #for deep copies
import io   #for utf-8 encoding
from autoGenerateHelper import Str2Latex, GenerateLatexStrKeywordExamples, ExtractExamplesWithKeyword, \
          RemoveIndentation, RSTheaderString, RSTlabelString, RSTinlineMath, RSTmath, RSTurl, RSTmarkup, RSTcodeBlock, \
          LatexString2RST

writeRST = True
addExampleReferences = True #costs lot of time
theDocDir = '../../../docs/theDoc/'
rstDir='../../../docs/RST/'
fileDir='../../pythonDev/exudyn/'
filesParsed=[
             'artificialIntelligence.py',
             'basicUtilities.py',
             'beams.py',
             'FEM.py',
             'graphicsDataUtilities.py',
             'GUI.py', 
             'interactive.py',
             'kinematicTree.py',
             'lieGroupBasics.py', #Stefan Holzinger
             'physics.py',
             'plot.py',
             'processing.py',
             'rigidBodyUtilities.py',
             'robotics/roboticsCore.py',
             'robotics/future.py',
             'robotics/models.py',
             'robotics/mobile.py',
             'robotics/motion.py',
             'robotics/special.py',
             'robotics/utilities.py',
             'signalProcessing.py',
             'solver.py',
             'utilities.py',
             #'lieGroupIntegration.py', #Stefan Holzinger
             ]

docuTags = ['classFunction','class','function','input','output','author','date','notes','example','status']
headerTags = ['Details','Author','Date','Copyright','References','Notes','Example']

#function = basic/brief notes on function
#additionally, there are the following dictionary items:
#  functionName (string), 
#  argumentsList (list of strings), 
#  defaultArgumentsList (list of strings in same order as argumentsList)

tagPreamble = '#**' #this must be given at beginning of any tag


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



#convert string into latex format, reagrind _ and {}
def ToLatex(s, replaceCurlyBracket=True): #replace _ and other symbols to fit into latex code
    if replaceCurlyBracket:
        s = s.replace('{','\\{')
        s = s.replace('}','\\}')
        s = s.replace('_','\\_')

    # s = s.replace('[','\\[')
    # s = s.replace(']','\\]')
    s = s.replace('&','\&')
    return s


#*****************************************************
#find identifier in line
def HasDocuIdentifier(lineString, identifier):
    s = lineString.strip() #erase spaces at beginning
    n = len(identifier)
    if len(s) >= n+len(tagPreamble): #additional symbols needed: #**function
        if s[0:n+len(tagPreamble)] == tagPreamble+identifier:
            return True
    return False

#*****************************************************
#check if is definition: def Function(): ...
def HasDefinitionIdentifier(lineString):
    s = lineString.strip() #erase spaces at beginning for classes
    if len(s) > 4: #def F
        if s[0:4] == 'def ':
            return True
    return False

#*****************************************************
#check if is class: class Object: ...
def HasClassIdentifier(lineString):
    s = lineString.strip() #erase spaces at beginning
    if len(s) > 6: #class X...
        if s[0:6] == 'class ':
            return True
    return False

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

#*****************************************************
#extract function arguments for function line without leading 'def '
def GetFunctionArguments(functionLine):
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
        defaultArgumentsList+=[defaultArg]
        
    return [functionName,argumentsList,defaultArgumentsList]

#*****************************************************
#parse file and extract list of dictionaries (every dict for one function)
def ParsePythonFile(fileName):
    fileLines = []
    file=open(fileName,'r') 
    fileLines = file.readlines()
    file.close()

    nLines = len(fileLines)
    lineCnt = 0
    
    functionList = []       #list of dictionaries of free functions or class
    functionDict = {}       #current dictionary of function (also for class functions) will contain all information for this function in the end

    classList = []          #current list of classes will contain dictionaries of {'funtionList':classFunctionList, 'className':name} in the end
    classFunctionList = []  #list of dictionaries of class functions
    classDict = {}          #current dictionary of class will contain all information for this class in the end
    
    verbose = False
    classMode = False       #do not start in class mode

    #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
    #print(headerDict)

    #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #run through file:
    while lineCnt < nLines:
        #read some identifier: function, class, classFunction
        if HasDocuIdentifier(fileLines[lineCnt],'function') or HasDocuIdentifier(fileLines[lineCnt],'class') or HasDocuIdentifier(fileLines[lineCnt],'classFunction'):
            
            #complete last function reading
            if len(functionDict) != 0 and not classMode:
                functionList += [copy.deepcopy(functionDict)] #store copy of previous dict
                if verbose: print("add function:", functionDict['functionName'])
            elif len(functionDict) != 0 and classMode:
                if verbose: 
                    print("class functionDict:", functionDict)
                if not 'functionName' in functionDict: #then this still contains class definitions
                    #add class notes or example
                    for (key,value) in functionDict.items():
                        classDict[key] = value
                else:
                    classFunctionList += [copy.deepcopy(functionDict)] #store copy of previous dict
                    if verbose: 
                        print("add class function:", functionDict['functionName'])
            functionDict = {}

            currentLine = fileLines[lineCnt]
            #terminate last class definition:
            if (HasDocuIdentifier(currentLine,'function') or HasDocuIdentifier(currentLine,'class')) and not HasDocuIdentifier(currentLine,'classFunction'):
               #as we see a function or a new class, class mode can be turned off:
                if verbose: print("line:",currentLine)
                if classMode and len(classDict) != 0:
                    classDict['functionList'] = copy.deepcopy(classFunctionList)
                    classList += [copy.deepcopy(classDict)]
                    if verbose: 
                        print("  write class: ",classDict, '\n')

                    classDict = {}
                    classFunctionList = []
                    if verbose: 
                        print("\nSTART new class")
                        # print("  classFunctionList =",classFunctionList)
                        # print("  classDict =",classDict, '\n')
                
                if HasDocuIdentifier(currentLine,'class'):
                    if verbose: print("==>switch to class mode")
                    classMode = True
                else:
                    if verbose: print("==>switch to function mode")
                    classMode = False 
                



            fillInMode = ''         #this is the place where to fill in the information
            currentInfo = ''        #string containing current tag information
            definitionFinished = False #signals that function has been completely parsed
            while lineCnt < nLines and not definitionFinished:
                line = fileLines[lineCnt].strip()
                newTag = False
                for tag in docuTags:
                    if not newTag and HasDocuIdentifier(line, tag):
                        if fillInMode != '': 
                            # if not classMode:
                            functionDict[fillInMode] = currentInfo #complete writing of previous tag information
                            # else:
                            #     classDict[fillInMode] = currentInfo #complete writing of previous tag information
                                
                        fillInMode = tag
                        newTag = True
                        currentInfo = line.split(tagPreamble+tag)[1]
                        if currentInfo[0] == ':': #erase ':', which may be omitted
                            currentInfo = currentInfo[1:]
                        currentInfo+="\n"
                #if fillInMode == 'example' and len(currentInfo) > 1: #always has length 1 ...
                    #print("example=", currentInfo)
                    #currentInfo = currentInfo.replace('{','\{').replace('}','\}')
                    #currentInfo = '\\begin{lstlisting}[language=Python]\n'+currentInfo+'\\end{lstlisting}' #' \\vspace{6pt}'
                #now add remaining line or total line
                if not newTag:
                    if len(line.strip()) > 1 and line.strip()[0] == '#': #comment needed to add into docu info
                        currentInfo += line.strip()[1:]+"\n"
                    elif HasDefinitionIdentifier(line): #parse arguments
                        if verbose: print("function def line:",line)
                        storedLineNumber = lineCnt #beginning of function definition, for hyperref
                        definitionFinished = True
                        if fillInMode != '': 
                            functionDict[fillInMode] = currentInfo #complete writing of previous tag information
                            fillInMode = ''
                        functionLine= line.strip()[4:] #without 'def '
                        while functionLine.strip()[-1] != ':': #read complete function definition
                            lineCnt+=1
                            functionLine += fileLines[lineCnt]
                        [functionName, argumentsList, defaultArgumentsList]=GetFunctionArguments(functionLine)
                        functionDict['functionName'] = Str2Latex(functionName)
                        functionDict['lineNumber'] = storedLineNumber
                        functionDict['argumentsList'] = argumentsList
                        functionDict['defaultArgumentsList'] = defaultArgumentsList
                    elif HasClassIdentifier(line): #parse arguments
                        if verbose: print("class def line:",line)
                        definitionFinished = True
                        if fillInMode != '': 
                            classDict[fillInMode] = currentInfo #complete writing of previous tag information
                            fillInMode = ''
                        className= line.strip()[6:-1] #without 'class ' and ':'

                        classDict['className'] = Str2Latex(className)
            
                lineCnt+=1 #increase lines while reading single function docu information
        else:
            #print("skip line:",fileLines[lineCnt])
            lineCnt+=1 #search for identifier
    

    #write last function dictionary:    
    if len(functionDict) != 0 and not classMode:
        functionList += [copy.deepcopy(functionDict)] #store copy of previous dict
        if verbose: print("add function:", functionDict['functionName'])
    elif len(functionDict) != 0 and classMode:
        classFunctionList += [copy.deepcopy(functionDict)] #store copy of previous dict
        if verbose: print("add class function:", functionDict['functionName'])

    #write last class:
    if classMode and len(classDict) != 0:
        if verbose: 
            print("  write class: ",classDict, '\n')
        classDict['functionList'] = copy.deepcopy(classFunctionList)
        classList += [copy.deepcopy(classDict)]
        classDict = {}
        classFunctionList = []

    return [functionList, classList, headerDict]

#*****************************************************
#convert tags of tagList in functionDict to latex and RST
mycnt = 0
def DictToItemsText(functionDict, tagList, addStr):
    global mycnt
    sLatex = ''
    sRST = ''
    sIndentRST = '  '
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
            if tag == 'example':
                #print("example=", strTag)
                sLatex += '\\vspace{-12pt}\\ei' #for global itemize list for function
                sLatex += '\\begin{lstlisting}[language=Python, xleftmargin=36pt]\n'
                sLatex += RemoveIndentation(strTag, '  ', False)
                if sLatex[-1] != '\n': sLatex+='\n'
                sLatex += '\\end{lstlisting}' #' \\vspace{6pt}'
                sLatex += '\\vspace{-24pt}\\bi\item[]\\vspace{-24pt}' #for global itemize list for function
                sRST += '\n'+RSTcodeBlock(RemoveIndentation(strTag.strip(), '  ', False)+'\n', 'python')
            elif strTag.count("\n") > 0 and strTag.strip() != '': #multiple lines are replaced by list
                sLatex += '\\vspace{-6pt}\n'+sSpaces+'\\begin{itemize}[leftmargin=1.2cm]\n'
                sLatex += '\setlength{\itemindent}{-0.7cm}\n'
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
                            sr = RSTmarkup(s[:n],'``') + LatexString2RSTspecial(s[n:], replaceMarkups = replaceMarkups) #in this string, there should be no markup ...
                            s = '{\\it '+s[:n]+'}'+ s[n:]
                        else:
                            sr = LatexString2RSTspecial(s, replaceMarkups = replaceMarkups)
                        sLatex += sSpaces*2+'\item[]'+s+'\n'
                        sRST += '\n  | '+RemoveIndentation(sr)
                    
                sLatex += '\\end{itemize}\n'
                #sRST += '\n'+RemoveIndentation(strTag.strip(), '  | ')
                sRST += '\n'
            else: #
                #sLatex += strTag.replace('\n','\\\\ \n') + '\n'
                sLatex += strTag.strip() + '\n' #in this case, we strip all spaces and newlines left, may be empty lines
                sRST += '  | ' + LatexString2RSTspecial(strTag, replaceMarkups = replaceMarkups).strip() + '\n'
    return [sLatex, sRST]

#*****************************************************
#write single function description into latex code
def WriteFunctionDescription2LatexRST(functionDict, moduleNamePython, pythonFileName, isClassFunction = False, className=''):
    sLatex = ''
    sRST = ''
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
        lineNumberStr = '\#L'+str(functionDict['lineNumber']+1)
    sLatex += '\\begin{flushleft}\n'
    #github link:
    url = 'https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/'+pythonFileName +lineNumberStr
    sLatex += '\\noindent '+addStr+'{def {\\bf \exuUrl{'+url
    sLatex += '}{' + functionName +'}{' '}}}'
    
    #relative file link:
    #sLatex += '\\noindent '+addStr+'{def \\mybold{\exuUrl{file:../../main/pythonDev/exudyn/' + moduleNamePython +'.py'+'}{' + functionName +'}{' '}}}'
    sLabel = 'sec:'+ moduleNamePython + ':' + classLabelStr + functionName.replace('\\_','_')
    sLatex += '\\label{'+sLabel+'}\n'

    sRST += RSTlabelString(sLabel.replace(':','-').replace('_','').lower())+'\n'

    #see also https://github.com/sphinx-doc/sphinx/issues/3921
    sRST += RSTurl(functionName, url, False) + '_\\ (' #add another _ to make url anonymous (otherwise warning, as function name my be duplicated)

    sLatex += '('
    sep = ''
    for i in range(len(argList)):
        if len(argList[i].strip()) != 0:
            sLatex += sep+'{\\it '+argList[i]+'}'
            sRST += sep + '\\ ``' + argList[i]
            if len(argDefault[i]) != 0:
                sLatex += '= '+argDefault[i]
                sRST += ' = '+argDefault[i] 
            sep = ', '
            sRST += '``\\ '
        
    [sDictLatex, sDictRST] = DictToItemsText(functionDict, docuTags, addStr)

    sLatex += ')\n'
    sLatex += '\\end{flushleft}\n'
    sLatex += '\setlength{\itemindent}{0.7cm}\n'
    sLatex += '\\begin{itemize}[leftmargin=0.7cm]\n'
    sLatex += sDictLatex
    sLatex += '\\vspace{12pt}\\end{itemize}\n%\n'
    
    sRST += ')\n\n' + sDictRST
    
    #sRST += DictToItemsText(functionDict, docuTags, addStr)
    
    return [sLatex,sRST]


#%%*****************************************************
#process files:
print('*****************************************')
print('create documentation for exudyn utilities')

#parse all files:
listRST = [] #creates tuple of modulename and RST content
sLatex = ''
for fileName in filesParsed:
    sRST = ''
    [functionList,classList,header] = ParsePythonFile(fileDir+fileName)
    moduleName = fileName[:-3]
    moduleNameLatex = moduleName.replace('robotics/roboticsCore','robotics').replace('/','.')
    
    moduleNamePython = moduleName.split('/')[-1]
    strSub = ''
    sectionLevel = 2
    if '.' in moduleNameLatex: #don't do it for robotics core 
        strSub = 'sub'
        sectionLevel += 1
        # print('found / in ', moduleName)
        # print('  =>'+'\\my'+strSub+'subsection{Module: '+moduleNameLatex+'}\n')
    sLatex += '\\my'+strSub+'subsection{Module: '+moduleNameLatex+'}\n'
    sLatex += '\\label{sec:module:'+moduleNameLatex+'}\n'

    sRST += RSTlabelString('sec-module-'+moduleNameLatex.replace('.','-'))+'\n'
    sRST += RSTheaderString('Module: '+moduleNameLatex, sectionLevel)+'\n'

    #*****************************************************
    if 'Details' in header: #write details as intro to section
        sLatex += header['Details'] #+ '\n'
        sRST += LatexString2RSTspecial(RemoveIndentation(header['Details']))
        #print('header=\n'+sRST)
    if len(header)>1:
        sLatex += '\\begin{itemize}[leftmargin=1.4cm]\n'
        sLatex += '\\setlength{\itemindent}{-1.4cm}\n'
        sRST += '\n'
        for tag in headerTags:
            if tag in header and tag != 'Details' and tag != 'Copyright':
                if header[tag].find('\\') == -1:
                    sLatex += '\\item[]' + tag + ': ' + header[tag] #+ '\n'
                    sRST += '- '+ tag + ': ' + LatexString2RSTspecial(header[tag].replace('\n',' ')) + '\n'
                else:
                    sRST += '- | ' + tag.strip() + ':'+'\n'
                    listString = header[tag].split('\\\\')
                    sLatex += '\\item[]' + tag + ':' + '\n' #+ listString[0] + ' \n'
                    sLatex += '\\vspace{-22pt}'
                    sLatex += '\\begin{itemize}[leftmargin=0.5cm]\n'
                    sLatex += '\\setlength{\itemindent}{-0.5cm}\n'
                    for i in range(len(listString)-0):
                        sTag = listString[i+0]
                        sLatex += '\\item[]' + sTag.replace('\n',' ') + '\n'
                        sRST += '  | '+ LatexString2RSTspecial(sTag.replace('\n',' ').strip()) + '\n'
                    sLatex += '\\ei\n'
                    
        sLatex += '\\ei\n'
    sRST += '\n'

    #*****************************************************
    isFirstFunction = True
    #insert function descriptions 
    for funcDict in functionList:
        if not isFirstFunction:
            sLatex += "\\noindent\\rule{8cm}{0.75pt}\\vspace{1pt} \\\\ \n"
            sRST += "\n----\n" #horizontal ruler
            #sLatex += "\\hline\\vspace{3pt}\\\\ \n"
        [sFuncLatex, sFuncRST] = WriteFunctionDescription2LatexRST(funcDict, moduleNamePython, fileName)
        sLatex += sFuncLatex
        sRST += sFuncRST
        
        #++++++++++++++++++++++++++++++++++++
        #add example references for function
        sExamples = ''
        sExamplesRST = ''
        if addExampleReferences:
            [sExamples,sExamplesRST] = GenerateLatexStrKeywordExamples('UtilityFunction', funcDict['functionName'], '', useLatex=False)
            sLatex += sExamples
            sRST += '\n'+sExamplesRST

        isFirstFunction=False

    #insert class descriptions with functions
    for classDict in classList:
        sLatex += '\\my'+strSub+'subsubsection{CLASS '+classDict['className']+' (in module '+moduleNameLatex+')}\n'
        #sLatex += '\\bi'
        sLatex += '\\noindent\\textcolor{steelblue}{{\\bf class description}}: ' + classDict['class']

        sRST += '\n' + RSTheaderString('CLASS '+classDict['className']+' (in module '+moduleNameLatex+')', sectionLevel+1)
        sRST += RSTmarkup('class description','**', False)+': ' + '\n\n' + \
            RemoveIndentation(LatexString2RSTspecial( classDict['class'] ), '    ') #+ '\n'


        #sLatex += '\\ei'
        localTags = docuTags.copy()
        localTags.remove('class')
        [sTags, sTagsRST] = DictToItemsText(classDict, localTags, '')
        if sTags != '':
            sLatex += '\setlength{\itemindent}{0.7cm}\n'
            sLatex += '\\begin{itemize}[leftmargin=0.7cm]\n'
            sLatex += sTags
            sLatex += '\\vspace{24pt}\\end{itemize}\n%\n'
            sRST += '\n' + sTagsRST + '\n'
        else:
            sLatex += '\\vspace{3pt} \\\\ \n' #add space for new class
            sRST += '\n'

        isFirstFunction = True
        for funcDict in classDict['functionList']:
            if not isFirstFunction:
                sLatex += "\\noindent\\rule{8cm}{0.75pt}\\vspace{1pt} \\\\ \n"
                sRST += "\n----\n" #horizontal ruler

                #sLatex += "\\hline\\vspace{3pt}\\\\ \n"
            [sFuncLatex, sFuncRST] = WriteFunctionDescription2LatexRST(funcDict, moduleNamePython, fileName, isClassFunction=True, className=classDict['className'])
            sLatex += sFuncLatex
            sRST += sFuncRST

            isFirstFunction=False

        #use split in class, for derived classes like InertiaCylinder(RigidBodyInertia)
        if addExampleReferences:
            [sExamples,sExamplesRST] = GenerateLatexStrKeywordExamples('UtilityFunction', classDict['className'].split('(')[0], '', useLatex=False)
            sLatex += sExamples
            sRST += '\n'+sExamplesRST

    sRST = sRST #.replace('**kwargs','\\*\\*kwargs').replace('*args','\\*args') #only needed, if not in literal
    #listRST += [(moduleNameLatex, LatexString2RSTspecial(sRST, replaceMarkups=False))]
    listRST += [(moduleNameLatex, sRST)]

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++        
latexFile = theDocDir+'pythonUtilitiesDescription.tex'
file=open(latexFile,'w')  #clear file by one write access
file.write('% ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++')
file.write('% description of python utility functions; generated by Johannes Gerstmayr')
file.write('% ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n\n')
file.write(sLatex)
file.close()


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++        
rstFile = rstDir+'pythonUtilities/pythonUtilities.rst'

sRSTpreamble = RSTlabelString('sec-pythonUtilityFunctions')
sRSTpreamble +="""
========================
Python Utility Functions
========================

This chapter describes in every subsection the functions and classes of the utility modules. 
These modules help to create multibody systems with the EXUDYN core module. Functions are implemented in Python and can be easily changed, extended and also verified by the user. **Check the source code** by entering these functions in Sypder and pressing ``CTRL + left mouse button``\ . These Python functions are much slower than the functions available in the C++ core. Some matrix computations with larger matrices implemented in numpy and scipy, however, are parallelized and therefore very efficient.

Note that in general functions accept lists and numpy arrays. If not, an error will occur, which is easily tracked.
Furthermore, angles are generally provided in radian ($2\pi$ equals $360\,^o$) and no units are used for distances, but it is recommended to use SI units (m, kg, s) throughout.

Functions have been implemented, if not otherwise mentioned, by Johannes Gerstmayr.
"""
sRSTpreamble = LatexString2RSTspecial(sRSTpreamble, replaceMarkups=False)


if writeRST:
    file=io.open(rstFile,'w',encoding='utf8')  #clear file by one write access
    file.write(sRSTpreamble)
    file.close()

    sRSTindex = RSTheaderString('Python Utility Functions',0)
    sRSTindex += """
.. toctree::
   :maxdepth: 2
   
   pythonUtilities
"""

    for (name, text) in listRST:
        file=io.open(rstDir+'pythonUtilities/'+name+'.rst','w',encoding='utf8')  #clear file by one write access
        file.write(text)
        file.close()
        sRSTindex += '   '+name+'\n'

    file=io.open(rstDir+'pythonUtilities/index.rst','w',encoding='utf8')  #clear file by one write access
    file.write(sRSTindex)
    file.close()
    





#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++        

print('----------- finished ---------------------')

