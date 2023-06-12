# -*- coding: utf-8 -*-
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN autogeneration file
#
# Details: 	used to create a single __init__.pyi file for the exudyn module;
#           alleviates auto-completion in Spyder or VScode
#
# Author:   Johannes Gerstmayr
# Date:     2023-05-09 (created)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import copy #for deep copies
import io   #for utf-8 encoding

rstFolder = 'docs/RST/' #folder where generated .rst files are stored

sourceDir=''
destFile=  '../../../main/pythonDev/exudyn/__init__.pyi'

#main files
filesParsed=[
              'stubHeader.pyi',
              'generated/stubEnums.pyi',
              'generated/stubSystemStructures.pyi',
              'generated/stubAutoBindings.pyi',
              'generated/stubAutoBindingsExt.pyi',
            ]

mergedFile=''
# for fileName in filesParsed:
#     file=open(sourceDir+fileName,'r') 
#     # fileLines = file.readlines()
#     mergedFile += file.read()
#     file.close()

#operate per class, to have only one structure per class!
classData = {}          #dictionary of class data
#mergedFileLines = []    #lines of all files
#lineCnt = 0
currentClass = ''
for fileName in filesParsed:
    #file=open(sourceDir+fileName,'r') 
    with open(sourceDir+fileName, 'r') as file:
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
    file=io.open(destFile,'w',encoding='utf8')
    file.write(mergedFile)
    file.close()




