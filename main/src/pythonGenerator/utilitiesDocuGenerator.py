# -*- coding: utf-8 -*-
"""
Created on Tue Jun 09 23:53:30 2020

@author: Johannes Gerstmayr

goal: generate latex documentation for all utilities packages
"""
import copy #for deep copies
from autoGenerateHelper import Str2Latex

fileDir='../../pythonDev/'
filesParsed=['exudynBasicUtilities.py',
             'exudynUtilities.py',
             'exudynGraphicsDataUtilities.py',
             'exudynRigidBodyUtilities.py',
             ##'exudynLieGroupIntegration.py',
             'exudynFEM.py'
             ]

docuTags = ['classFunction','class','function','input','output','author','notes','example']
#function = basic/brief notes on function
#additionally, there are the following dictionary items:
#  functionName (string), 
#  argumentsList (list of strings), 
#  defaultArgumentsList (list of strings in same order as argumentsList)

tagPreamble = '#**' #this must be given at beginning of any tag

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

    currentString = ''
    for c in s:
        if c == ',' and (bracket0+bracket1+bracket2+bracket3) == 0:
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

    strList += [currentString]
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
    
    #run through file:
    while lineCnt < nLines:
        #read some identifier: function, class, classFunction
        if HasDocuIdentifier(fileLines[lineCnt],'function') or HasDocuIdentifier(fileLines[lineCnt],'class') or HasDocuIdentifier(fileLines[lineCnt],'classFunction'):
            
            #complete last function reading
            if len(functionDict) != 0 and not classMode:
                functionList += [copy.deepcopy(functionDict)] #store copy of previous dict
                if verbose: print("add function:", functionDict['functionName'])
            elif len(functionDict) != 0 and classMode:
                classFunctionList += [copy.deepcopy(functionDict)] #store copy of previous dict
                if verbose: 
                    print("class functionDict:", functionDict)
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
                #now add remaining line or total line
                if not newTag:
                    if len(line.strip()) > 1 and line.strip()[0] == '#': #comment needed to add into docu info
                        currentInfo += line.strip()[1:]+"\n"
                    elif HasDefinitionIdentifier(line): #parse arguments
                        if verbose: print("function def line:",line)
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

    return [functionList, classList]

#*****************************************************
#write single function description into latex code
def WriteFunctionDescription2Latex(functionDict, isClassFunction = False):
    sLatex = ''
    argList = functionDict['argumentsList']
    argDefault = functionDict['defaultArgumentsList']
    addStr = ''
    if isClassFunction:
        addStr = '\\textcolor{steelblue}'
    
    sLatex += '\\noindent '+addStr+'{def {\\bf ' + functionDict['functionName']+'}}'
    sLatex += '('
    sep = ''
    for i in range(len(argList)):
        sLatex += sep+'{\\it '+argList[i]+'}'
        if len(argDefault[i]) != 0:
            sLatex += '='+argDefault[i]
        sep = ', '
        
    sLatex += ')\n'
    sLatex += '\setlength{\itemindent}{0.7cm}\n'
    sLatex += '\\begin{itemize}[leftmargin=0.7cm]\n'
    for tag in docuTags:
        if tag in functionDict:
            text = tag
            if tag == 'function':
                text = 'function description'
            if tag == 'class':
                text = 'class description'
            sLatex += '  \\item[--]  '+addStr+'{\\bf ' + text + '}: '
            strTag = functionDict[tag].strip()
            #print(strTag)
            if strTag.count("\n") > 0: #multiple lines are replaced by list
                sLatex += '\\vspace{-6pt}\n  \\begin{itemize}[leftmargin=1.2cm]\n'
                sLatex += '\setlength{\itemindent}{-0.7cm}\n'
                if strTag[0:2] == '\n':
                    strTag = strTag[2:]
                if strTag[-2:] == '\n':
                    strTag = strTag[:-2]

                strTagList = strTag.split('\n')
                #replace words with ':' with italic characters
                for s in strTagList:
                    if s.find(':') != -1: #first occurance
                        n=s.find(':')
                        s = '{\\it '+s[:n]+'}'+ s[n:]
                    sLatex += '    \item[] '+s+'\n'
                    
                sLatex += '  \\ei\n'
            else:
                sLatex += strTag.replace('\n','\\\\ \n')

    sLatex += '\\vspace{12pt}\\end{itemize}\n%\n'
    
    return sLatex



#*****************************************************
#process files:


#parse all files:
sLatex = ''
for fileName in filesParsed:
    [functionList,classList] = ParsePythonFile(fileDir+fileName)
    moduleName = fileName[:-3]
    sLatex += '\\mysubsection{Module: '+moduleName+'}\n'
    isFirstFunction = True
    #insert function descriptions
    for funcDict in functionList:
        if not isFirstFunction:
            sLatex += "\\noindent\\rule{8cm}{0.75pt}\\vspace{1pt} \\\\ \n"
            #sLatex += "\\hline\\vspace{3pt}\\\\ \n"
        sLatex += WriteFunctionDescription2Latex(funcDict)
        isFirstFunction=False

    #insert class descriptions with functions
    for classDict in classList:
        sLatex += '\\mysubsubsection{CLASS '+classDict['className']+' (in module '+moduleName+')}\n'
        #sLatex += '\\bi'
        sLatex += '\\noindent\\textcolor{steelblue}{{\\bf class description}}: ' + classDict['class'] + '\\vspace{9pt} \\\\ \n'
        #sLatex += '\\ei'

        isFirstFunction = True
        for funcDict in classDict['functionList']:
            if not isFirstFunction:
                sLatex += "\\noindent\\rule{8cm}{0.75pt}\\vspace{1pt} \\\\ \n"
                #sLatex += "\\hline\\vspace{3pt}\\\\ \n"
            sLatex += WriteFunctionDescription2Latex(funcDict, isClassFunction=True)
            isFirstFunction=False

        
latexFile = '..\\..\\..\\docs\\theDoc\\pythonUtilitiesDescription.tex'

file=open(latexFile,'w')  #clear file by one write access
file.write('% ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++')
file.write('% description of python utility functions; generated by Johannes Gerstmayr')
file.write('% ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n\n')
file.write(sLatex)
file.close()

        
        