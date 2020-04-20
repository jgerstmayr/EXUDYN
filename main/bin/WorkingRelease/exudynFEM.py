#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  support functions for finite element models
#
# Author:   Johannes Gerstmayr
# Date:     2020-03-10
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++import sys
#constants and fixed structures:
import numpy as np #LoadSolutionFile

#convert sparse matrix data to dense numpy matrix
def CompressedRowToDenseMatrix(sparseData):
    n = int(np.max(sparseData[:,0])) #rows and columns are 1-based
    m = np.zeros((n,n))
    for row in sparseData:
        m[int(row[0])-1,int(row[1])-1] = row[2]
    return m

#read abaqus nodes information to numpy array
#typeName is Part or Instance; name is part's or instance's name
#if exportElement=False: returns np.array(nodes) with nodal coordinates
#if exportElements=True: returns a list [np.array(nodes), elementsDict, surfaceElementsDict] with information on types of elements
def ReadNodesFromAbaqusInp(fileName, typeName='Part', name='Part-1', exportElements=False):
    fileLines = []
    try: #still close file if crashes
        file=open(fileName,'r') 
        fileLines = file.readlines()
    finally:
        file.close()
        
        
    print("read ", len(fileLines), "lines")
    
    startPart = False
    startReadNodes = False
    finishedReadNodes = False
    startReadElements = False
    finishedReadElements = False
    nodes = [] #store list of node values
    elements = [] #store list of elements with node numbers
    surfaceElements = [] #store list of surface elements (trigs, quads) with node numbers
    elementTypes = [] #string list of element types
    surfaceElementTypes = [] #string list of surface element types

    errorOccured = False
    lineCnt = 0
    for line in fileLines:
        #print("line", lineCnt, "=", line)
        lineCnt+=1
        if errorOccured:
            break
        
        if startReadNodes and not finishedReadNodes:
            if line[0] != '*': #check if nodes section has finished
                lineData = line.split(',') #split into values
                if len(lineData) != 4:
                    print("ERROR: Expected node number and 3 coordinates, line ", lineCnt)
                    errorOccured = True
                else:
                    v = []
                    for i in range(3):
                        v+=[float(lineData[i+1])]
                    nodes += [v] #add node data
            else:
                startReadNodes = False
                finishedReadNodes = True
    
        if startReadElements and not finishedReadElements:
            if line[0] != '*': #check if nodes section has finished
                lineData = line.split(',') #split into values
                if len(lineData) != 9:
                    print("ERROR: Expected element number and 8 indices for C3D8R, line=", lineCnt)
                    errorOccured = True
                else:
                    v = []
                    for i in range(8):
                        v+=[float(lineData[i+1])]
                    elements += [v] #add node data
                    elementTypes += ['Hex8'] #8-noded hexahedral
            else:
                startReadElements = False
                finishedReadElements = True
    
        if startPart and not startReadNodes and not finishedReadNodes:
            if line[0:5] == '*Node':
                startReadNodes = True
                startPart = False
            else:
                print("ERROR: Expected *Node after *Part, line=", lineCnt)
                errorOccured = True

        if finishedReadNodes and exportElements and not startReadElements:
            if line[0:8] == '*Element':
                startReadElements = True
                #check "type=C3D8R" in future

            
        if line[0:len(typeName)+1] == '*'+typeName:
            if not startPart:
                #print("ERROR: only one *Part section allowed, line=", lineCnt)
                #errorOccured = True
                
                lineInfo = line.split(',')
                #print(lineInfo)
                if len(lineInfo) != 3:
                    print("ERROR: invalid information for part/instance name, line=", lineCnt)
                    errorOccured = True
                else:
                    nameInfo = lineInfo[1].strip().split('=')
                    if nameInfo[0] != 'name':
                        print("ERROR: expected 'name=' in line=", lineCnt)
                        errorOccured = True
                    else:
                        if nameInfo[1] != name:
                            print("ERROR: expected name='" + name + "' in line=", lineCnt)
                            errorOccured = True
                        else:
                            startPart = True
    
    if not exportElements:
        return np.array(nodes)
    else:
        elementsDict = {'elements':np.array(elements), 'elementTypes':elementTypes}
        surfaceElementsDict = {'elements':np.array(surfaceElements), 'elementTypes':surfaceElementTypes}
        return [np.array(nodes), elementsDict, surfaceElementsDict]

#convert list of C3D8 Hex element with 8 nodes in nodeNumbers into triangle-List
def ConvertHexToTrigs(nodeNumbers):
    localList = [[0,1,2], [0,2,3], [6,5,4], [6,4,7], [0,4,1], [1,4,5], [1,5,2], [2,5,6], [2,6,3], [3,6,7], [3,7,0], [0,7,4]]

    trigList = []
    for trig in localList:
        ind = [nodeNumbers[trig[0]], nodeNumbers[trig[1]], nodeNumbers[trig[2]]]
        trigList += [ind]
    return trigList
