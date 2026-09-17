# -*- coding: utf-8 -*-
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN autogeneration file
#
# Details: 	used to create a single __init__.pyi file for the exudyn module;
#           alleviates auto-completion in Spyder or VScode;
#           also create a symbolic.pyi for the submodule
#
# Author:   Johannes Gerstmayr
# Date:     2023-05-09 (created)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import copy #for deep copies
import io   #for utf-8 encoding
import ast  #to verify that the written stub is valid Python

#stub files are merged from fragments line by line; a fragment that indents differently
#than expected silently produces a file that no longer parses, and then no IDE and no type
#checker can read it - therefore every written stub is parsed before it is accepted, #2486
def CheckStubIsValidPython(fileName, text):
    try:
        ast.parse(text, filename=fileName)
    except SyntaxError as e:
        raise SyntaxError('createStubFiles.py produced an invalid stub file "'+fileName+
                          '", line '+str(e.lineno)+': '+str(e.msg)+
                          '\n  '+str(e.text).rstrip()+
                          '\nthe stub fragments are merged by indentation; a class docstring '
                          'line starting at column 0 ends the class block, see issue #2486') from None

rstFolder = 'docs/RST/' #folder where generated .rst files are stored

sourceDir=''
import generatorPaths as paths
destFile=  paths.pythonPackageDir+'__init__.pyi'

#main files
filesParsed=[
              paths.generatorDir+'stubHeader.pyi',
              paths.generatedDir+'stubEnums.pyi',
              paths.generatedDir+'stubSystemStructures.pyi',
              paths.generatedDir+'stubAutoBindings.pyi',
              paths.generatedDir+'stubAutoBindingsExt.pyi',
            ]

mergedFile=''

#operate per class, to have only one structure per class!
classData = {}          #dictionary of class data
#mergedFileLines = []    #lines of all files
#lineCnt = 0
currentClass = ''
for fileName in filesParsed:
    with open(sourceDir+fileName, 'r', encoding='utf8') as file:
        fileLines = file.readlines()
        for line in fileLines:
            if line[0:6] == 'class ':
                className = line.split(' ')[1].split(':')[0].strip()
                currentClass = className
                #print(className)
                if className not in classData:
                    classData[className] = '\n'+line #initiate string
                # else:
                #     print('class', className, 'already exists')
            elif line[0:4] != ' '*4 and line.strip() != '':
                currentClass = '' #something at first column, so class has ended
                #print('class end:', line)

            if currentClass!='':
                if line[0] != '#' and line[0:6] != 'class ': #class line may not be added again!!!; comments are likely at the wrong place
                    classData[currentClass] += line
                    #print('class', currentClass,'add:', line)
            else:
                #print('no class add:', line)
                if line[0] != '#': #these comments are likely at the wrong place
                    mergedFile += line
                
            #mergedFileLines += [line]

for key, value in classData.items():
    mergedFile += value
            
# for line in mergedFileLines:
#     mergedFile += line

if True:
    CheckStubIsValidPython(destFile, mergedFile) #raises before anything is written, #2486
    file=io.open(destFile,'w',encoding='utf8')
    file.write(mergedFile)
    file.close()




if True: 
    destFile2=  paths.pythonPackageDir+'symbolic.pyi'

    file=io.open(paths.generatorDir+'stubHeader.pyi','r',encoding='utf8')  
    mergedText = file.read()
    file.close()

    file=io.open(paths.generatedDir+'stubSymbolic.pyi','r',encoding='utf8')  
    mergedText += file.read()
    file.close()

    CheckStubIsValidPython(destFile2, mergedText) #raises before anything is written, #2486
    file=io.open(destFile2,'w',encoding='utf8')
    file.write(mergedText)
    file.close()


